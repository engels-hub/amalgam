//! [`MeshOpTool`]: the hand-built Phase 1 dialog of every mesh op (S07 Refinement log, owner
//! question 4), on the framework API: UI-03 tool + [`PreviewDriver`](crate::preview::PreviewDriver),
//! UI-04 selection (a forwarded [`SelectTool`]), UI-15 widgets.
//!
//! The dialog edits [`OpParams`]; whenever the resulting feature changes it is previewed through
//! `Document::preview` (a debounced, latest-wins, cancellable job; inputs above the proxy limit
//! are previewed on a decimated proxy, `amg_features::mesh::uses_preview_proxy`). OK commits
//! the same `AddFeature` as one undo entry; the full compute then runs as the document's
//! recompute job. Nothing heavy runs on the UI thread.

use std::borrow::Cow;
use std::sync::Arc;

use amg_doc::{BodyId, Command as DocCommand, Diagnostic, Evaluated, FeatureBox, Health};
use amg_jobs::JobState;
use amg_mesh::IndexedMesh;

use super::analysis::{body_label, mesh_bodies, mesh_of};
use super::ops::{MeshOp, OpParams};
use crate::app::{CmdEnv, CommitBatch};
use crate::command::{CmdId, Enablement};
use crate::input::ViewportEvent;
use crate::pick::PickTarget;
use crate::preview::PreviewState;
use crate::select::{EntityFilter, SelItem, SelectTool};
use crate::tool::{FinishReason, InputResult, Tool, ToolCx, ToolInit};
use crate::widgets::{DialogAction, DialogFrame, SelectionField, toggle};

/// Label of the document's preview job (`Document::preview`).
const PREVIEW_JOB: &str = "Preview";

/// What the preview was last requested for.
#[derive(Clone, Debug, PartialEq)]
struct PreviewKey {
    body: BodyId,
    mesh: usize,
    params: OpParams,
    faces: Vec<u32>,
    new_body: bool,
}

/// The dialog tool of one mesh op.
pub struct MeshOpTool {
    op: MeshOp,
    params: OpParams,
    picker: SelectTool,
    bodies: Vec<SelItem>,
    bodies_active: bool,
    faces: Vec<SelItem>,
    faces_active: bool,
    new_body: bool,
    /// The body the length defaults were sized for.
    fitted: Option<BodyId>,
    /// What the preview shows (or computes); `None` = no preview requested.
    previewed: Option<PreviewKey>,
}

impl std::fmt::Debug for MeshOpTool {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("MeshOpTool")
            .field("op", &self.op)
            .field("params", &self.params)
            .field("bodies", &self.bodies.len())
            .field("faces", &self.faces.len())
            .finish_non_exhaustive()
    }
}

impl MeshOpTool {
    /// The tool of `op`, taking the preselected mesh body and faces. Without a preselected
    /// body, the design's only mesh body is used.
    pub fn new(op: MeshOp, init: &ToolInit<'_>) -> MeshOpTool {
        let ev = init.env.evaluated;
        let is_mesh = |t: &PickTarget| t.body().is_some_and(|b| mesh_of(ev, b).is_some());
        let mut bodies: Vec<SelItem> = init
            .preselection
            .iter()
            .filter(|i| {
                matches!(i.target, PickTarget::Body { sub: None, .. }) && is_mesh(&i.target)
            })
            .take(1)
            .cloned()
            .collect();
        let faces: Vec<SelItem> = if op.uses_faces() {
            init.preselection
                .iter()
                .filter(|i| {
                    matches!(
                        i.target,
                        PickTarget::Body {
                            sub: Some((amg_doc::TopoKind::Face, _)),
                            ..
                        }
                    ) && is_mesh(&i.target)
                })
                .cloned()
                .collect()
        } else {
            Vec::new()
        };
        if bodies.is_empty() && faces.is_empty() {
            let mut all = mesh_bodies(ev);
            if let (Some(only), None) = (all.next(), all.next())
                && let Some(item) = SelItem::from_target(PickTarget::Body {
                    body: only,
                    instance: 0,
                    sub: None,
                })
            {
                bodies.push(item);
            }
        }
        let faces_active = op.uses_faces() && !bodies.is_empty() && faces.is_empty();
        MeshOpTool {
            op,
            params: OpParams::new(op),
            picker: SelectTool::default(),
            bodies_active: bodies.is_empty() && faces.is_empty(),
            bodies,
            faces,
            faces_active,
            new_body: false,
            fitted: None,
            previewed: None,
        }
    }

    /// The op.
    pub fn op(&self) -> MeshOp {
        self.op
    }

    /// The dialog state (tests and scripting read it).
    pub fn params(&self) -> &OpParams {
        &self.params
    }

    /// The target body: the body field, else the body of the first selected face.
    fn body(&self) -> Option<BodyId> {
        self.bodies
            .iter()
            .chain(&self.faces)
            .find_map(|i| i.target.body())
    }

    fn face_indices(&self, body: BodyId) -> Vec<u32> {
        let mut v: Vec<u32> = self
            .faces
            .iter()
            .filter_map(|i| match i.target {
                PickTarget::Body {
                    body: b,
                    sub: Some((amg_doc::TopoKind::Face, f)),
                    ..
                } if b == body => Some(f),
                _ => None,
            })
            .collect();
        v.sort_unstable();
        v.dedup();
        v
    }

    /// The feature the dialog describes now, or why there is none.
    fn build(&self, env: &CmdEnv<'_>) -> Result<(FeatureBox, PreviewKey), Cow<'static, str>> {
        let body = self.body().ok_or("Select a mesh body")?;
        let mesh = mesh_of(env.evaluated, body).ok_or("The selected body is not a mesh")?;
        let faces = self.face_indices(body);
        if self.op.needs_faces() && faces.is_empty() {
            return Err("Select the faces to erase".into());
        }
        let prefs = &env.design.meta().default_units;
        if let Some(e) = self.params.first_error(env.design.params(), prefs) {
            return Err(e.into());
        }
        let feature = self
            .params
            .feature(self.op, body, &mesh, &faces, self.new_body)?;
        Ok((
            feature,
            PreviewKey {
                body,
                mesh: Arc::as_ptr(&mesh) as usize,
                params: self.params.clone(),
                faces,
                new_body: self.new_body,
            },
        ))
    }

    fn add_command(env: &CmdEnv<'_>, feature: FeatureBox) -> DocCommand {
        DocCommand::AddFeature {
            at: None,
            feature,
            context: env.active_component,
            name: None,
        }
    }
}

/// The health of the feature a preview added (the node the authored design does not have).
fn preview_health(env: &CmdEnv<'_>, ev: &Evaluated) -> Option<Health> {
    let timeline = env.design.timeline();
    ev.nodes()
        .find(|(id, _)| timeline.node(*id).is_none())
        .map(|(_, n)| n.health.clone())
}

/// Progress of the running preview job, `0..=1`.
fn preview_progress(env: &CmdEnv<'_>) -> Option<f32> {
    env.jobs
        .jobs()
        .into_iter()
        .find(|j| j.label == PREVIEW_JOB)
        .map(|j| match j.state {
            JobState::Running(p) => p.fraction,
            _ => 0.0,
        })
}

impl Tool for MeshOpTool {
    fn id(&self) -> CmdId {
        self.op.cmd()
    }

    fn label(&self) -> Cow<'static, str> {
        self.op.title().into()
    }

    fn input(&mut self, ev: &ViewportEvent, cx: &mut ToolCx<'_>) -> InputResult {
        self.picker.input(ev, cx)
    }

    fn update(&mut self, cx: &mut ToolCx<'_>) {
        let built = {
            let env = cx.env();
            self.build(&env)
                .map(|(f, key)| (Self::add_command(&env, f), key))
        };
        match built {
            Ok((cmd, key)) => {
                if self.previewed.as_ref() != Some(&key) {
                    self.previewed = Some(key);
                    cx.preview().set(cmd);
                }
            }
            Err(_) => {
                if self.previewed.take().is_some() {
                    cx.preview().clear();
                }
            }
        }
    }

    fn dialog_ui(&mut self, ui: &mut egui::Ui, cx: &mut ToolCx<'_>) -> bool {
        // Everything read from the document is taken first: the fields borrow the selection.
        let state = cx.preview().state().clone();
        let (params, prefs, mesh, ok, health, computing, progress) = {
            let env = cx.env();
            let mesh: Option<(BodyId, Arc<IndexedMesh>)> = self
                .body()
                .and_then(|b| mesh_of(env.evaluated, b).map(|m| (b, m)));
            let ok = match self.build(&env) {
                Ok(_) => Enablement::Enabled,
                Err(why) => Enablement::Disabled(why),
            };
            let health = match &state {
                PreviewState::Ready(ev) => preview_health(&env, ev),
                PreviewState::Failed(m) => Some(Health::Error(Diagnostic {
                    message: m.clone(),
                    cause: None,
                    entities: Vec::new(),
                })),
                _ => None,
            };
            let computing = matches!(state, PreviewState::Pending | PreviewState::Computing);
            (
                env.design.params().clone(),
                env.design.meta().default_units.clone(),
                mesh,
                ok,
                health,
                computing,
                preview_progress(&env),
            )
        };
        if let Some((b, m)) = &mesh
            && self.fitted != Some(*b)
        {
            self.fitted = Some(*b);
            self.params.fit_to(m);
        }
        let op = self.op;
        let before = self.params.clone();
        let mut cancel_preview = false;
        let action = DialogFrame::new(op.title())
            .banner(health.as_ref())
            .ok_enabled(ok)
            .show_inside(ui, |ui| {
                SelectionField::new(
                    "Body",
                    &mut self.bodies,
                    EntityFilter::BODY,
                    &mut self.bodies_active,
                )
                .limits(if op.uses_faces() { 0 } else { 1 }, Some(1))
                .mirror(cx.selection_mut())
                .show(ui);
                if op.uses_faces() {
                    SelectionField::new(
                        "Faces",
                        &mut self.faces,
                        EntityFilter::FACE,
                        &mut self.faces_active,
                    )
                    .limits(u32::from(op.needs_faces()), None)
                    .mirror(cx.selection_mut())
                    .show(ui);
                }
                if let Some((b, m)) = &mesh {
                    ui.weak(format!(
                        "Body {}: {} faces",
                        body_label(*b),
                        m.face_count()
                    ));
                }
                self.params.ui(ui, &params, &prefs);
                if op.has_output_mode() {
                    toggle(ui, "Create new body", &mut self.new_body);
                }
                if let Some((_, m)) = &mesh
                    && amg_features::mesh::uses_preview_proxy(m.face_count())
                {
                    let (_, target) = amg_features::mesh::preview_proxy_limits();
                    ui.weak(format!(
                        "Preview on a decimated proxy of about {target} faces; OK computes the full mesh."
                    ));
                }
                if computing {
                    ui.horizontal(|ui| {
                        ui.spinner();
                        match progress {
                            Some(p) if p > 0.0 => {
                                ui.add(
                                    egui::ProgressBar::new(p.clamp(0.0, 1.0))
                                        .desired_width(120.0)
                                        .text("Preview"),
                                );
                            }
                            _ => {
                                ui.label("Computing preview…");
                            }
                        }
                        if ui.button("Stop preview").clicked() {
                            cancel_preview = true;
                        }
                    });
                }
            });
        if let Some((_, m)) = &mesh {
            self.params.plane_changed(&before, m);
        }
        if cancel_preview {
            // Cancels the job; the preview restarts only when a field changes.
            cx.preview().clear();
        }
        match action {
            DialogAction::Ok => cx.request_finish(FinishReason::Commit),
            DialogAction::Cancel => cx.request_finish(FinishReason::Cancel),
            DialogAction::None => {}
        }
        true
    }

    fn pick_filter(&self) -> EntityFilter {
        if self.op.uses_faces() {
            EntityFilter::BODY | EntityFilter::FACE
        } else {
            EntityFilter::BODY
        }
    }

    fn status_hint(&self) -> Option<Cow<'static, str>> {
        Some(
            format!(
                "{}: set the options, then OK (Enter) or Esc",
                self.op.title()
            )
            .into(),
        )
    }

    fn finish(&mut self, reason: FinishReason, cx: &mut ToolCx<'_>) -> Option<CommitBatch> {
        if reason != FinishReason::Commit {
            return None;
        }
        let env = cx.env();
        let (feature, _) = self.build(&env).ok()?;
        Some(CommitBatch {
            label: format!("Mesh {}", self.op.title()),
            commands: vec![Self::add_command(&env, feature)],
        })
    }
}
