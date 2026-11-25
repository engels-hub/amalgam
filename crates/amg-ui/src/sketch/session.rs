//! The sketch session (spec §13 *Session*, *Edits*, *Threading*) and its document layer.
//!
//! [`SketchLayer`] is a per-document [`DocLayer`]: it owns the open [`SketchSession`], opens it
//! when the shell asks to edit a sketch (`DocTab::sketch_edit`, set by the browser, timeline
//! and Create Sketch), ends it when the `sketch` context goes away, and every frame polls the
//! solver worker, keeps the working copy in step with the document and draws the sketch
//! overlays.
//!
//! **Working copy.** The session edits a clone of the [`SketchFeature`]. Each edit is applied
//! to the clone and issued as one `EditFeature` (one undo entry). The design's feature is
//! watched by pointer: when it changes outside the session (undo / redo, or the command layer
//! promoting dimensions to model parameters), the working copy is reloaded from it.
//!
//! **Solver.** One [`SolverState`] per session moves into each [`SolverJob`] and back, so at
//! most one solver call is in flight. Every edit asks for a solve (latest wins: edits made
//! while one runs fold into the next); drag targets are coalesced (latest wins).

use std::collections::BTreeSet;
use std::sync::Arc;

use amg_core::{Plane, Point2};
use amg_doc::{Command, Expr, ExprSrc, Feature, FeatureBox, FeatureId, SketchId};
use amg_features::sketch::{SketchEdit, SketchFeature};
use amg_jobs::{JobHandle, JobState, Priority, SpawnOptions};
use amg_sketch::solve::{ConstraintCheck, ConstraintState, DofReport, DragSpec, SolveStatus};
use amg_sketch::{
    ConstraintId, ConstraintKind, EditResult, EntityId, EntityKind, ProfileSet, Quantity, Sketch,
    SketchError,
};

use super::dimension::dimension_text_expr;
use super::solver::{SolverJob, SolverJobError, SolverOp, SolverOut, SolverState};
use crate::app::{AppCx, CommitBatch};
use crate::command::CmdId;
use crate::layer::{DocLayer, DocLayerDef, LayerOut};
use crate::nav::CameraControl as _;

/// The palette's display and snap options (Sketch Palette).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PaletteOptions {
    /// Draw the grid on the sketch plane.
    pub grid: bool,
    /// Snap to the grid (when no inference applies).
    pub snap: bool,
    /// Shade closed profiles.
    pub show_profile: bool,
    /// Draw sketch points.
    pub show_points: bool,
    /// Draw dimension labels.
    pub show_dimensions: bool,
    /// Draw constraint glyphs.
    pub show_constraints: bool,
    /// Cut the view at the sketch plane.
    pub slice: bool,
}

impl Default for PaletteOptions {
    fn default() -> Self {
        PaletteOptions {
            grid: true,
            snap: false,
            show_profile: true,
            show_points: true,
            show_dimensions: true,
            show_constraints: true,
            slice: false,
        }
    }
}

/// Counters for tests and the report.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct SessionStats {
    /// Solves finished.
    pub solves: u32,
    /// Drag frames finished.
    pub drag_frames: u32,
    /// Edits issued to the document.
    pub commits: u32,
    /// Working-copy reloads from the document (undo / redo / external edits).
    pub reloads: u32,
}

/// Follow-up edits of a [`SketchSession::commit_op`].
#[derive(Clone, Debug, Default)]
pub struct Inferred {
    /// Added in the same undo entry (constraints that cannot over-constrain, dimensions).
    pub edits: Vec<SketchEdit>,
    /// Checked by `check_constraint` in a solver job first; only independent ones are added.
    pub checked: Vec<ConstraintKind>,
}

/// What the in-flight job is.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum JobKind {
    Solve,
    Check,
    Probe,
    DragStep,
    DragFinish,
}

struct Inflight {
    handle: JobHandle<(SolverState, SolverOut), SolverJobError>,
    /// The session generation the job's input belongs to.
    generation: u64,
    kind: JobKind,
}

#[derive(Debug)]
struct DragCtl {
    spec: DragSpec,
    begun: bool,
    target: Option<Point2>,
    release: bool,
    /// A frame failed to start: the drag ends without a commit.
    failed: bool,
}

/// A constraint or dimension waiting for its SK-04 classification (UI-14).
#[derive(Debug)]
struct Probe {
    label: String,
    kind: ConstraintKind,
    /// `Some(expr)`: a driving dimension with this value expression (refused ones are offered
    /// as driven); `None`: a geometric constraint (refused ones are not added).
    dimension: Option<Expr>,
    sent: bool,
}

/// A dimension that would over-constrain the sketch, offered as a driven dimension
/// ("Create as driven dimension?", QUESTIONS #16).
#[derive(Clone, Debug, PartialEq)]
pub struct DrivenOffer {
    /// The undo label of the dimension.
    pub label: String,
    /// The dimension (driving; it becomes driven on Yes).
    pub kind: ConstraintKind,
    /// `true` if it contradicts the sketch, `false` if it is implied (redundant).
    pub conflicting: bool,
    /// The other constraints of its dependency group.
    pub with: Vec<ConstraintId>,
}

#[derive(Debug)]
struct PendingCommit {
    label: String,
    candidates: Vec<ConstraintKind>,
    sent: bool,
}

/// An open sketch: working copy, solver worker, DOF report and palette.
pub struct SketchSession {
    feature: FeatureId,
    plane: Plane,
    working: SketchFeature,
    /// The design's feature this working copy was loaded from (pointer identity).
    synced: Option<Arc<dyn Feature>>,
    /// The newest solved (or, right after an edit, the edited) sketch: drawn and queried.
    display: Arc<Sketch>,
    dof: Option<DofReport>,
    status: Option<SolveStatus>,
    profiles: Arc<ProfileSet>,
    state: Option<SolverState>,
    /// Generation of the sketch inside `state`.
    state_gen: u64,
    inflight: Option<Inflight>,
    /// Bumped by every edit and reload; results of older jobs are dropped.
    generation: u64,
    want_solve: bool,
    drag: Option<DragCtl>,
    pending: Option<PendingCommit>,
    probe: Option<Probe>,
    /// The session's number in its layer (`SketchLayer::opened`); its dialogs compare it.
    serial: u32,
    /// The dimension label being dragged and where it is now (not committed yet).
    label_drag: Option<(ConstraintId, Point2)>,
    /// Display and snap options.
    pub palette: PaletteOptions,
    /// Construction mode: new geometry is construction geometry (`X`).
    pub construction: bool,
    /// Selected sketch entities.
    pub selected: BTreeSet<EntityId>,
    /// The entity under the cursor (set by the sketch tools each frame).
    pub hovered: Option<EntityId>,
    /// The active creation tool's newest inference (glyph, guides; tests and scripts).
    pub inference: Option<super::infer::Inference>,
    /// The active creation tool's open heads-up input (tests and scripts).
    pub heads_up: Option<super::tools::HeadsUp>,
    /// The constraint whose glyph (or dimension whose label) is under the cursor (UI-14).
    pub hovered_constraint: Option<ConstraintId>,
    /// Selected constraint glyphs and dimension labels (Delete removes them; UI-14).
    pub selected_constraints: BTreeSet<ConstraintId>,
    /// Why the newest constraint or dimension was not added (status bar; UI-14).
    pub notice: Option<String>,
    /// An over-constraining dimension waiting for "Create as driven dimension?" (UI-14).
    pub driven_offer: Option<DrivenOffer>,
    /// The newest dimension added through [`add_dimension`](Self::add_dimension) or
    /// [`accept_driven`](Self::accept_driven) (its tool opens the value editor on it).
    pub last_dimension: Option<ConstraintId>,
    /// The newest operation of a modify tool (UI-16; tests and scripts replay it).
    pub last_modify: Option<super::modify::ModifyOp>,
    stats: SessionStats,
}

impl std::fmt::Debug for SketchSession {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SketchSession")
            .field("feature", &self.feature)
            .field("generation", &self.generation)
            .field("dof", &self.dof.as_ref().map(|d| d.dof))
            .field("stats", &self.stats)
            .finish_non_exhaustive()
    }
}

fn thin(a: &Arc<dyn Feature>) -> *const () {
    Arc::as_ptr(a).cast::<()>()
}

impl SketchSession {
    /// Opens feature `id` of the document; `None` if it is not a sketch.
    fn open(cx: &mut AppCx<'_>, id: FeatureId) -> Option<SketchSession> {
        let env = cx.env();
        let (_, node) = env.design.timeline().node(id)?;
        let feat = node.feature.downcast_ref::<SketchFeature>()?.clone();
        let synced = Some(Arc::clone(&node.feature.0));
        let geom = env.evaluated.sketch(SketchId(id)).cloned();
        let plane = geom
            .as_ref()
            .map_or_else(|| origin_plane(&feat).unwrap_or(Plane::XY), |g| g.plane);
        let mut display = feat.sketch.clone();
        display.set_plane(plane);
        let mut working = feat;
        working.sketch.set_plane(plane);
        if let Some(g) = &geom
            && g.sketch.params().len() == working.sketch.params().len()
        {
            // Start from the solved values (dimension values follow their expressions there).
            display = (*g.sketch).clone();
        }
        Some(SketchSession {
            feature: id,
            plane,
            state: Some(SolverState::new(display.clone())),
            state_gen: 0,
            display: Arc::new(display),
            working,
            synced,
            dof: None,
            status: None,
            profiles: Arc::new(ProfileSet::default()),
            inflight: None,
            generation: 0,
            want_solve: true,
            drag: None,
            pending: None,
            probe: None,
            serial: 0,
            label_drag: None,
            palette: PaletteOptions::default(),
            construction: false,
            selected: BTreeSet::new(),
            hovered: None,
            inference: None,
            heads_up: None,
            hovered_constraint: None,
            selected_constraints: BTreeSet::new(),
            notice: None,
            driven_offer: None,
            last_dimension: None,
            last_modify: None,
            stats: SessionStats::default(),
        })
    }

    /// The sketch feature being edited.
    pub fn feature(&self) -> FeatureId {
        self.feature
    }

    /// The resolved sketch plane.
    pub fn plane(&self) -> &Plane {
        &self.plane
    }

    /// The working copy.
    pub fn working(&self) -> &SketchFeature {
        &self.working
    }

    /// The sketch that is drawn and queried (newest solve, or the edited copy before it).
    pub fn display(&self) -> &Arc<Sketch> {
        &self.display
    }

    /// The newest DOF report (`None` before the first solve finished).
    pub fn dof(&self) -> Option<&DofReport> {
        self.dof.as_ref()
    }

    /// The newest solve status.
    pub fn status(&self) -> Option<SolveStatus> {
        self.status
    }

    /// The profiles of the newest solve.
    pub fn profiles(&self) -> &Arc<ProfileSet> {
        &self.profiles
    }

    /// Counters.
    pub fn stats(&self) -> SessionStats {
        self.stats
    }

    /// The DOF colour class of entity `e` (`None` before the first solve or for unknown ids).
    pub fn entity_state(&self, e: EntityId) -> Option<ConstraintState> {
        self.dof.as_ref()?.entities.get(&e).map(|s| s.state)
    }

    /// A solver job is running or queued work waits (tests, idle detection).
    pub fn is_solving(&self) -> bool {
        self.inflight.is_some()
            || self.want_solve
            || self.pending.is_some()
            || self.drag.is_some()
            || self.probe.is_some()
    }

    /// An edit waits for its constraint check, or a drag is open: tools hold new input.
    pub fn is_busy(&self) -> bool {
        self.pending.is_some() || self.drag.is_some() || self.probe.is_some()
    }

    /// The session's number in its layer (dialogs that belong to it compare it).
    pub fn serial(&self) -> u32 {
        self.serial
    }

    /// The entities drawn highlighted because a constraint glyph or dimension label is hovered
    /// or selected: exactly the entities those constraints reference.
    pub fn highlighted(&self) -> BTreeSet<EntityId> {
        let mut out = BTreeSet::new();
        for id in self
            .hovered_constraint
            .iter()
            .chain(&self.selected_constraints)
        {
            if let Some(c) = self.display.constraint(*id) {
                out.extend(c.kind.entities());
            }
        }
        out
    }

    /// The dimension label being dragged and its current position.
    pub fn label_drag(&self) -> Option<(ConstraintId, Point2)> {
        self.label_drag
    }

    /// Adds geometric constraint `kind` after an SK-04 check in a solver job (one undo entry
    /// labelled `label`). A redundant or conflicting one is not added; [`notice`](Self::notice)
    /// says why. Until the check is back [`is_busy`](Self::is_busy) is true.
    ///
    /// # Errors
    /// The session is busy, or `kind` is invalid for the sketch (wrong entity kinds).
    pub fn add_constraint(
        &mut self,
        cx: &mut AppCx<'_>,
        label: &str,
        kind: ConstraintKind,
    ) -> Result<(), SketchError> {
        self.probe_then(cx, label, kind, None)
    }

    /// Adds a driving dimension (`kind` must be a `ConstraintKind::Dimension`) with value
    /// expression `expr` after an SK-04 check. When it would over-constrain the sketch it is
    /// not added: [`driven_offer`](Self::driven_offer) is set and the "Create as driven
    /// dimension?" prompt opens ([`accept_driven`](Self::accept_driven)).
    ///
    /// # Errors
    /// The session is busy, or `kind` is not a valid dimension of the sketch.
    pub fn add_dimension(
        &mut self,
        cx: &mut AppCx<'_>,
        label: &str,
        kind: ConstraintKind,
        expr: Expr,
    ) -> Result<(), SketchError> {
        if !matches!(kind, ConstraintKind::Dimension(_)) {
            return Err(SketchError::NotApplicable("not a dimension"));
        }
        self.probe_then(cx, label, kind, Some(expr))
    }

    fn probe_then(
        &mut self,
        cx: &mut AppCx<'_>,
        label: &str,
        kind: ConstraintKind,
        dimension: Option<Expr>,
    ) -> Result<(), SketchError> {
        if self.is_busy() {
            return Err(SketchError::NotApplicable("an edit is still being checked"));
        }
        // Reject wrong entity kinds now (the rules `Sketch::add_constraint` applies).
        let mut probe = self.working.sketch.clone();
        probe.add_constraint(kind.clone())?;
        self.notice = None;
        self.driven_offer = None;
        self.probe = Some(Probe {
            label: label.to_owned(),
            kind,
            dimension,
            sent: false,
        });
        cx.request_repaint();
        Ok(())
    }

    /// Answers the "Create as driven dimension?" prompt: `yes` adds the offered dimension as a
    /// driven (measured) dimension in one undo entry, `no` drops it.
    pub fn accept_driven(&mut self, cx: &mut AppCx<'_>, yes: bool) {
        let Some(offer) = self.driven_offer.take() else {
            return;
        };
        self.notice = None;
        if !yes {
            return;
        }
        let ConstraintKind::Dimension(mut d) = offer.kind else {
            return;
        };
        d.driving = false;
        let edit = SketchEdit::AddConstraint {
            kind: ConstraintKind::Dimension(d),
            expr: None,
        };
        match self.commit(cx, &offer.label, edit, Vec::new()) {
            Ok(r) => self.last_dimension = r.constraints_added.first().copied(),
            Err(e) => self.notice = Some(format!("Not added: {e}")),
        }
    }

    /// Removes constraints and dimensions `ids` (one undo entry).
    ///
    /// # Errors
    /// The session is busy or an id is unknown.
    pub fn remove_constraints(
        &mut self,
        cx: &mut AppCx<'_>,
        ids: &[ConstraintId],
    ) -> Result<(), SketchError> {
        if ids.is_empty() {
            return Ok(());
        }
        let edits = ids
            .iter()
            .map(|&id| SketchEdit::RemoveConstraint(id))
            .collect();
        let label = if ids.len() == 1 {
            "Delete constraint"
        } else {
            "Delete constraints"
        };
        self.commit(cx, label, SketchEdit::Batch(edits), Vec::new())?;
        self.selected_constraints.retain(|c| !ids.contains(c));
        if self.hovered_constraint.is_some_and(|c| ids.contains(&c)) {
            self.hovered_constraint = None;
        }
        Ok(())
    }

    /// The value expression of driving dimension `id` as the editor shows it: the expression
    /// of its model parameter (`d3`), or its inline text.
    pub fn dimension_source(&self, design: &amg_doc::Design, id: ConstraintId) -> Option<String> {
        match self.working.dims.get(&id)? {
            Expr::Param(p) => design.params().get(*p).map(|p| p.expr.0.clone()),
            Expr::Inline(src) => Some(src.0.clone()),
        }
    }

    /// The model parameter behind driving dimension `id`, if it has one (`d<n>`).
    pub fn dimension_param(&self, id: ConstraintId) -> Option<amg_doc::ParamId> {
        match self.working.dims.get(&id)? {
            Expr::Param(p) => Some(*p),
            Expr::Inline(_) => None,
        }
    }

    /// Sets the value expression of driving dimension `id` to `src` (one undo entry). A
    /// dimension backed by a model parameter (`d<n>`, the usual case) gets the new expression
    /// on that parameter (`SetParamExpr`, so the parameter keeps its name); one with an inline
    /// expression gets `SketchEdit::SetDimension`. A bare number takes mm or degrees. The
    /// working copy picks up the new value and re-solves.
    ///
    /// # Errors
    /// The message of the check that failed (unknown name, unit, cycle, a driven dimension);
    /// nothing changed.
    pub fn set_dimension_expr(
        &mut self,
        cx: &mut AppCx<'_>,
        id: ConstraintId,
        src: &str,
    ) -> Result<(), String> {
        let q = match self.working.sketch.constraint(id).map(|c| &c.kind) {
            Some(ConstraintKind::Dimension(d)) if d.driving => d.kind.quantity(),
            Some(ConstraintKind::Dimension(_)) => {
                return Err("a driven dimension only measures".to_owned());
            }
            _ => return Err("not a dimension".to_owned()),
        };
        let text = {
            let env = cx.env();
            let (params, prefs) = (env.design.params(), &env.design.meta().default_units);
            dimension_text_expr(params, prefs, src, quantity_dim(q))?
        };
        if let Some(p) = self.dimension_param(id) {
            let cmd = Command::SetParamExpr { id: p, expr: text };
            let mut copy = cx.env().design.clone();
            amg_doc::apply_command(&mut copy, cmd.clone()).map_err(|e| e.to_string())?;
            cx.submit(CommitBatch {
                label: "Edit dimension".to_owned(),
                commands: vec![cmd],
            });
            self.stats.commits += 1;
            cx.request_repaint();
            return Ok(());
        }
        let edit = SketchEdit::SetDimension {
            id,
            expr: Expr::Inline(ExprSrc(text)),
        };
        self.commit(cx, "Edit dimension", edit, Vec::new())
            .map(|_| ())
            .map_err(|e| e.to_string())
    }

    /// Starts dragging the label of dimension `id` (no undo entry until
    /// [`end_label_drag`](Self::end_label_drag)).
    pub fn begin_label_drag(&mut self, id: ConstraintId) -> bool {
        if self.is_busy() {
            return false;
        }
        let Some(d) = self
            .display
            .constraint(id)
            .and_then(|c| c.kind.as_dimension())
        else {
            return false;
        };
        self.label_drag = Some((id, d.label_pos));
        true
    }

    /// Moves the dragged label to `pos` (sketch coordinates).
    pub fn drag_label_to(&mut self, pos: Point2) {
        if let Some((_, p)) = &mut self.label_drag {
            *p = pos;
        }
    }

    /// Drops the dragged label: one `SetDimensionLabel` undo entry (S03 R5).
    pub fn end_label_drag(&mut self, cx: &mut AppCx<'_>) {
        let Some((id, pos)) = self.label_drag.take() else {
            return;
        };
        let moved = self
            .working
            .sketch
            .constraint(id)
            .and_then(|c| c.kind.as_dimension())
            .is_some_and(|d| d.label_pos != pos);
        if moved {
            let edit = SketchEdit::SetDimensionLabel { id, pos };
            if let Err(e) = self.commit(cx, "Move dimension label", edit, Vec::new()) {
                tracing::debug!("label move skipped: {e}");
            }
        }
    }

    /// Writes the current values of the driving dimensions' expressions (a parameter edited
    /// in Change Parameters, an edited dimension) into the working copy; true if one changed
    /// (the sketch then re-solves).
    fn refresh_dimension_values(&mut self, cx: &AppCx<'_>) -> bool {
        let env = cx.env();
        let (params, prefs) = (env.design.params(), &env.design.meta().default_units);
        let mut changed = Vec::new();
        for (id, e) in &self.working.dims {
            let Some(ConstraintKind::Dimension(d)) =
                self.working.sketch.constraint(*id).map(|c| &c.kind)
            else {
                continue;
            };
            let Ok(v) = params.resolve(e, quantity_dim(d.kind.quantity()), prefs) else {
                continue;
            };
            if v.value.is_finite() && v.value.to_bits() != d.value.to_bits() {
                changed.push((*id, v.value));
            }
        }
        for (id, v) in &changed {
            let _ = self.working.sketch.set_dimension_value(*id, *v);
        }
        !changed.is_empty()
    }

    /// A drag is open.
    pub fn is_dragging(&self) -> bool {
        self.drag.is_some()
    }

    /// Applies `edit` to the working copy now and issues it as one undo entry labelled `label`.
    ///
    /// `checked` are inferred constraints that might over-constrain the sketch: they are
    /// checked in a solver job first (SK-04 `check_constraint`) and only the independent ones
    /// are added, in the same undo entry. Until then [`is_busy`](Self::is_busy) is true.
    ///
    /// # Errors
    /// The edit is invalid; nothing changed.
    pub fn commit(
        &mut self,
        cx: &mut AppCx<'_>,
        label: &str,
        edit: SketchEdit,
        checked: Vec<ConstraintKind>,
    ) -> Result<EditResult, SketchError> {
        self.commit_op(
            cx,
            label,
            |f| f.apply(edit),
            |_, _| Inferred {
                edits: Vec::new(),
                checked,
            },
        )
    }

    /// Like [`commit`](Self::commit) for an edit whose follow-up constraints depend on the
    /// ids it creates: `op` edits the feature (an [`SketchEdit`] or a builder through
    /// `SketchFeature::edit_sketch`), then `then` returns the edits to add in the same undo
    /// entry (invalid ones are skipped) and the constraints to check first.
    ///
    /// # Errors
    /// `op` failed; nothing changed.
    pub fn commit_op(
        &mut self,
        cx: &mut AppCx<'_>,
        label: &str,
        op: impl FnOnce(&mut SketchFeature) -> Result<EditResult, SketchError>,
        then: impl FnOnce(&EditResult, &Sketch) -> Inferred,
    ) -> Result<EditResult, SketchError> {
        if self.pending.is_some() {
            return Err(SketchError::NotApplicable("an edit is still being checked"));
        }
        let mut work = self.working.clone();
        let res = op(&mut work)?;
        let Inferred { edits, checked } = then(&res, &work.sketch);
        for e in edits {
            if let Err(err) = work.apply(e) {
                tracing::debug!("inferred sketch edit skipped: {err}");
            }
        }
        self.working = work;
        self.edited();
        if checked.is_empty() {
            self.submit(cx, label);
        } else {
            self.pending = Some(PendingCommit {
                label: label.to_owned(),
                candidates: checked,
                sent: false,
            });
        }
        cx.request_repaint();
        Ok(res)
    }

    /// The working copy changed: show it now, solve it next.
    fn edited(&mut self) {
        self.generation += 1;
        self.want_solve = true;
        self.display = Arc::new(self.working.sketch.clone());
        let d = &self.display;
        self.selected.retain(|e| d.entity(*e).is_some());
    }

    /// Issues the working copy as `EditFeature` (one undo entry).
    fn submit(&mut self, cx: &mut AppCx<'_>, label: &str) {
        cx.submit(CommitBatch {
            label: label.to_owned(),
            commands: vec![Command::EditFeature {
                id: self.feature,
                feature: FeatureBox::new(self.working.clone()),
            }],
        });
        // Reload from whatever the design holds next frame (also after a failed edit).
        self.synced = None;
        self.stats.commits += 1;
    }

    /// The Construction command (`X`): turns the selected curves into construction geometry,
    /// or back to normal when all of them are construction already (one undo entry). With no
    /// curve selected it flips [`construction`](Self::construction), the mode of new geometry.
    ///
    /// # Errors
    /// The sketch is busy with an earlier edit.
    pub fn toggle_construction(&mut self, cx: &mut AppCx<'_>) -> Result<(), SketchError> {
        let curves: Vec<(EntityId, bool)> = self
            .selected
            .iter()
            .filter_map(|&id| {
                let e = self.working.sketch.entity(id)?;
                (!matches!(e.kind, EntityKind::Point { .. })).then_some((id, e.construction))
            })
            .collect();
        if curves.is_empty() {
            self.construction = !self.construction;
            cx.request_repaint();
            return Ok(());
        }
        let on = !curves.iter().all(|(_, c)| *c);
        let edits = curves
            .into_iter()
            .map(|(id, _)| SketchEdit::SetConstruction { id, on })
            .collect();
        let batch = SketchEdit::Batch(edits);
        self.commit(cx, "Toggle construction", batch, Vec::new())?;
        Ok(())
    }

    /// Starts dragging `entity` grabbed at `grab` (sketch coordinates). False if the entity
    /// cannot be dragged or the session is busy.
    pub fn begin_drag(&mut self, entity: EntityId, grab: Point2) -> bool {
        if self.is_busy() {
            return false;
        }
        let draggable = matches!(
            self.display.entity(entity).map(|e| &e.kind),
            Some(
                EntityKind::Point { .. }
                    | EntityKind::Line { .. }
                    | EntityKind::Circle { .. }
                    | EntityKind::Arc { .. }
            )
        );
        if !draggable {
            return false;
        }
        self.drag = Some(DragCtl {
            spec: DragSpec { entity, grab },
            begun: false,
            target: Some(grab),
            release: false,
            failed: false,
        });
        true
    }

    /// Moves the open drag toward `target` (coalesced: only the newest target is solved).
    pub fn drag_to(&mut self, target: Point2) {
        if let Some(d) = &mut self.drag {
            d.target = Some(target);
        }
    }

    /// Releases the drag: the last frame finishes, then the result becomes one `CommitDrag`.
    pub fn end_drag(&mut self) {
        if let Some(d) = &mut self.drag {
            d.release = true;
        }
    }

    /// Per frame: resync with the document, collect the finished job, start the next one.
    fn poll(&mut self, cx: &mut AppCx<'_>) -> bool {
        // The feature still exists?
        let node_feature = {
            let env = cx.env();
            env.design
                .timeline()
                .node(self.feature)
                .map(|(_, n)| Arc::clone(&n.feature.0))
        };
        let Some(node_feature) = node_feature else {
            return false;
        };
        let external = self
            .synced
            .as_ref()
            .is_none_or(|s| thin(s) != thin(&node_feature));
        if external && self.pending.is_none() && self.drag.is_none() && self.probe.is_none() {
            match node_feature.as_any().downcast_ref::<SketchFeature>() {
                Some(f) => {
                    let mut w = f.clone();
                    w.sketch.set_plane(self.plane);
                    self.working = w;
                    self.synced = Some(node_feature);
                    self.stats.reloads += 1;
                    self.edited();
                }
                None => return false,
            }
        }
        if self.pending.is_none()
            && self.drag.is_none()
            && self.probe.is_none()
            && self.refresh_dimension_values(cx)
        {
            self.edited();
        }
        self.collect(cx);
        self.schedule(cx);
        if self.inflight.is_some() || self.want_solve {
            cx.request_repaint();
        }
        true
    }

    fn collect(&mut self, cx: &mut AppCx<'_>) {
        let Some(job) = &mut self.inflight else {
            return;
        };
        let Some(res) = job.handle.try_take() else {
            return;
        };
        let (generation, kind) = (job.generation, job.kind);
        self.inflight = None;
        let (state, out) = match res {
            Ok(v) => v,
            Err(e) => {
                tracing::warn!("sketch solver job ended: {e}");
                // The state is lost with the job: start over from the working copy.
                self.state = Some(SolverState::new(self.working.sketch.clone()));
                self.state_gen = self.generation;
                self.want_solve = true;
                if kind != JobKind::Solve {
                    self.drag = None;
                    self.pending = None;
                    self.probe = None;
                }
                return;
            }
        };
        self.state = Some(state);
        self.state_gen = generation;
        match out {
            SolverOut::Solved { report, profiles } => {
                self.stats.solves += 1;
                if generation != self.generation {
                    return;
                }
                let st = self.state.as_ref().map(|s| &s.sketch);
                if let Some(solved) = st {
                    if report.status.is_converged()
                        && solved.params().len() == self.working.sketch.params().len()
                    {
                        // Keep the solved values as the warm start of the working copy.
                        for (i, v) in solved.params().as_slice().iter().enumerate() {
                            let p = amg_sketch::ParamId(i as u32);
                            self.working.sketch.params_mut().set(p, *v);
                        }
                    }
                    self.display = Arc::new(solved.clone());
                }
                self.status = Some(report.status);
                self.dof = report.dof;
                self.profiles = profiles;
            }
            SolverOut::Checked(accepted) => {
                let Some(p) = self.pending.take() else {
                    return;
                };
                let mut edits = Vec::new();
                for (k, ok) in p.candidates.into_iter().zip(accepted) {
                    if ok {
                        edits.push(SketchEdit::AddConstraint {
                            kind: k,
                            expr: None,
                        });
                    }
                }
                if !edits.is_empty() && self.working.apply(SketchEdit::Batch(edits)).is_ok() {
                    self.edited();
                }
                self.submit(cx, &p.label);
            }
            SolverOut::Probed(res) => {
                let Some(p) = self.probe.take() else {
                    return;
                };
                self.finish_probe(cx, p, res);
            }
            SolverOut::DragStepped(step) => {
                self.stats.drag_frames += 1;
                match step {
                    Ok(_) => {
                        if let Some(s) = &self.state {
                            self.display = Arc::new(s.sketch.clone());
                        }
                    }
                    Err(e) => {
                        tracing::debug!("drag did not start: {e}");
                        if let Some(d) = &mut self.drag {
                            d.failed = true;
                        }
                    }
                }
            }
            SolverOut::DragFinished(report) => {
                self.drag = None;
                let Some(s) = &self.state else { return };
                let (old, new) = (self.working.sketch.params(), s.sketch.params());
                let values: Vec<_> = (0..old.len().min(new.len()))
                    .filter_map(|i| {
                        let p = amg_sketch::ParamId(i as u32);
                        (old.get(p).to_bits() != new.get(p).to_bits()).then(|| (p, new.get(p)))
                    })
                    .collect();
                if values.is_empty() || !report.status.is_converged() {
                    // Nothing moved, or the final solve failed: show the working copy again.
                    self.want_solve = true;
                    self.generation += 1;
                    return;
                }
                if self
                    .working
                    .apply(SketchEdit::CommitDrag { values })
                    .is_ok()
                {
                    self.edited();
                    self.submit(cx, "Drag sketch geometry");
                }
            }
        }
    }

    /// Adds a probed constraint or dimension, or says why not and offers a dimension as driven.
    fn finish_probe(
        &mut self,
        cx: &mut AppCx<'_>,
        p: Probe,
        res: Result<ConstraintCheck, SketchError>,
    ) {
        let (conflicting, with) = match res {
            Ok(ConstraintCheck::Independent) => {
                let edit = SketchEdit::AddConstraint {
                    kind: p.kind,
                    expr: p.dimension.clone(),
                };
                match self.working.apply(edit) {
                    Ok(r) => {
                        if p.dimension.is_some() {
                            self.last_dimension = r.constraints_added.first().copied();
                        }
                        self.edited();
                        self.submit(cx, &p.label);
                    }
                    Err(e) => self.notice = Some(format!("Not added: {e}")),
                }
                return;
            }
            Ok(ConstraintCheck::Redundant { with }) => (false, with),
            Ok(ConstraintCheck::Conflicting { with }) => (true, with),
            Err(e) => {
                self.notice = Some(format!("Not added: {e}"));
                return;
            }
        };
        if p.dimension.is_some() {
            self.notice = Some("the dimension would over-constrain the sketch".to_owned());
            self.driven_offer = Some(DrivenOffer {
                label: p.label,
                kind: p.kind,
                conflicting,
                with,
            });
            cx.open_dialog(Box::new(super::dimension::DrivenPromptDialog {
                session: self.serial,
            }));
        } else {
            let why = if conflicting {
                "conflicts with existing constraints"
            } else {
                "is already implied by existing constraints"
            };
            self.notice = Some(format!("Not added: {} {why}", p.kind.name()));
        }
        cx.request_repaint();
    }

    fn schedule(&mut self, cx: &mut AppCx<'_>) {
        if self.inflight.is_some() {
            return;
        }
        let Some(mut state) = self.state.take() else {
            return;
        };
        let op = if let Some(p) = self.pending.as_mut().filter(|p| !p.sent) {
            p.sent = true;
            Some((
                SolverOp::Check(Box::new(self.working.sketch.clone()), p.candidates.clone()),
                JobKind::Check,
            ))
        } else if self.pending.is_some() {
            None
        } else if let Some(p) = self.probe.as_mut().filter(|p| !p.sent) {
            p.sent = true;
            Some((
                SolverOp::Probe(Box::new(self.working.sketch.clone()), p.kind.clone()),
                JobKind::Probe,
            ))
        } else if self.probe.is_some() {
            None
        } else if self.want_solve && (self.state_gen != self.generation || self.drag.is_none()) {
            self.want_solve = false;
            Some((
                SolverOp::Solve(Box::new(self.working.sketch.clone())),
                JobKind::Solve,
            ))
        } else if let Some(d) = &mut self.drag {
            if d.failed {
                self.drag = None;
                None
            } else if let Some(target) = d.target.take() {
                let begin = (!d.begun).then_some(d.spec);
                d.begun = true;
                Some((SolverOp::DragStep { begin, target }, JobKind::DragStep))
            } else if d.release {
                if d.begun {
                    Some((SolverOp::DragFinish, JobKind::DragFinish))
                } else {
                    self.drag = None;
                    None
                }
            } else {
                None
            }
        } else {
            None
        };
        let Some((op, kind)) = op else {
            self.state = Some(state);
            return;
        };
        if kind == JobKind::DragStep && self.state_gen != self.generation {
            // The solver's copy is older than the working copy (should not happen: a solve
            // runs first); drag on the current geometry.
            state.sketch = (*self.display).clone();
        }
        let handle = cx.jobs().spawn_with(
            SolverJob { state, op },
            SpawnOptions {
                priority: Priority::Interactive,
                hidden: true,
            },
        );
        self.inflight = Some(Inflight {
            handle,
            generation: self.generation,
            kind,
        });
    }

    /// Cancels the running job (the session is closing).
    fn cancel(&mut self) {
        if let Some(j) = &self.inflight {
            j.handle.cancel();
        }
    }

    /// The job state of the in-flight job (tests).
    pub fn job_state(&self) -> Option<JobState> {
        self.inflight.as_ref().map(|j| j.handle.state())
    }
}

/// The S01 dimension of a sketch dimension's value.
pub(crate) fn quantity_dim(q: Quantity) -> amg_core::Dimension {
    match q {
        Quantity::Length => amg_core::Dimension::LENGTH,
        Quantity::Angle => amg_core::Dimension::ANGLE,
    }
}

/// The plane of an origin-plane sketch (before its first recompute).
fn origin_plane(f: &SketchFeature) -> Option<Plane> {
    use amg_features::sketch::PlaneInput;
    match f.plane {
        PlaneInput::OriginXY => Some(Plane::XY),
        PlaneInput::OriginYZ => Some(Plane::YZ),
        PlaneInput::OriginXZ => Some(Plane::XZ),
        PlaneInput::Ref(_) => None,
    }
}

/// The per-document sketch layer: owns the open session.
#[derive(Debug, Default)]
pub struct SketchLayer {
    session: Option<SketchSession>,
    /// A sketch created by Create Sketch, opened once the document has it.
    pub(crate) created: Option<std::rc::Rc<std::cell::Cell<Option<FeatureId>>>>,
    /// Sessions opened so far (tests).
    opened: u32,
    /// Leave the session at the next update (Finish Sketch, Esc).
    finish: bool,
}

/// The layer registration.
pub const SKETCH_LAYER: DocLayerDef = DocLayerDef {
    id: "sketch.session",
    create: || Box::new(SketchLayer::default()),
};

impl SketchLayer {
    /// The open session.
    pub fn session(&self) -> Option<&SketchSession> {
        self.session.as_ref()
    }

    /// The open session, mutable.
    pub fn session_mut(&mut self) -> Option<&mut SketchSession> {
        self.session.as_mut()
    }

    /// Sessions opened in this document so far.
    pub fn opened(&self) -> u32 {
        self.opened
    }

    /// Leaves sketch mode at the next frame (Finish Sketch).
    pub fn request_finish(&mut self) {
        self.finish = true;
    }

    fn end(&mut self, cx: &mut AppCx<'_>) {
        if let Some(mut s) = self.session.take() {
            s.cancel();
            tracing::debug!(feature = ?s.feature, "sketch session ended");
        }
        self.finish = false;
        cx.exit_context(crate::design::SKETCH_CONTEXT);
        cx.request_repaint();
    }

    fn start(&mut self, cx: &mut AppCx<'_>, id: FeatureId) {
        if self.session.as_ref().is_some_and(|s| s.feature == id) {
            return;
        }
        if self.session.is_some() {
            self.end(cx);
        }
        match SketchSession::open(cx, id) {
            Some(mut s) => {
                cx.camera().look_at_plane(&s.plane, true);
                self.opened += 1;
                s.serial = self.opened;
                self.session = Some(s);
                cx.enter_context(crate::design::SKETCH_CONTEXT);
                cx.open_dialog(Box::new(super::palette::SketchPaletteDialog {
                    session: self.opened,
                }));
                cx.open_dialog(Box::new(super::glyphs::GlyphPainter {
                    session: self.opened,
                }));
            }
            None => cx.exit_context(crate::design::SKETCH_CONTEXT),
        }
    }
}

impl DocLayer for SketchLayer {
    fn update(&mut self, cx: &mut AppCx<'_>, out: &mut LayerOut<'_>) {
        if let Some(id) = cx.take_sketch_edit() {
            self.start(cx, id);
        }
        if let Some(id) = self.created.as_ref().and_then(|c| c.get()) {
            self.created = None;
            self.start(cx, id);
        }
        let in_context = cx.env().contexts.contains(&crate::design::SKETCH_CONTEXT);
        if self.session.is_some() && (self.finish || !in_context) {
            self.end(cx);
        }
        {
            let env = cx.env();
            let editing = self.session.as_ref().map(|s| s.feature);
            super::view::draw_finished(out.overlays, env.design, env.evaluated, editing);
        }
        let Some(s) = &mut self.session else {
            return;
        };
        if !s.poll(cx) {
            // The feature is gone (deleted, or its creation undone).
            self.end(cx);
            return;
        }
        super::view::draw_session(s, out.overlays, cx.doc_tab().view.as_ref());
        if s.palette.slice {
            out.sections
                .planes
                .push(amg_render::SectionPlane::new(s.plane));
        }
        if out.active_tool.is_none() {
            cx.run_command(SKETCH_SELECT_CMD);
        }
    }
}

/// The session's default tool (select and drag), restarted whenever no tool is active.
pub const SKETCH_SELECT_CMD: CmdId = CmdId("sketch.select");

/// Runs `f` with the active document's open session (taken out of its layer for the call, so
/// `f` can use the [`AppCx`]). `None` without a session.
pub(crate) fn with_session<R>(
    cx: &mut AppCx<'_>,
    f: impl FnOnce(&mut SketchSession, &mut AppCx<'_>) -> R,
) -> Option<R> {
    let mut s = cx.doc_layer::<SketchLayer>()?.session.take()?;
    let r = f(&mut s, cx);
    if let Some(l) = cx.doc_layer::<SketchLayer>() {
        l.session = Some(s);
    }
    Some(r)
}
