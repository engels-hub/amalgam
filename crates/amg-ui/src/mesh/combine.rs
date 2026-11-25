//! [`CombineTool`]: the Combine dialog (MESH-27), hand-built like the other Phase 1 mesh
//! dialogs ([`MeshOpTool`](super::tool::MeshOpTool)) but with two body fields: the mesh
//! **Target** and the **Tools** (mesh or solid bodies).
//!
//! - Join / Cut / Intersect, Keep tools, Create new body; the `mesh.combine` feature is
//!   previewed through `Document::preview` (latest-wins, cancellable job) and OK commits one
//!   undoable `AddFeature` ("Mesh Combine").
//! - A solid tool is converted to mesh: the dialog says so (and the feature warns), only for
//!   this mesh target.
//! - An implicit body as target or tool: a boolean with an implicit body stays implicit
//!   (QUESTIONS #63), which is S08's Implicit Combine. The dialog disables OK with that reason
//!   and shows no "converted to mesh" note.

use std::borrow::Cow;

use amg_doc::{BodyGeom, BodyId, Command as DocCommand, Diagnostic, Evaluated, FeatureBox, Health};
use amg_jobs::JobState;

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

/// Shown (and the OK reason) when an operand is implicit.
pub const IMPLICIT_NOTE: &str =
    "A combine with an implicit body stays implicit: use Implicit Combine (Design workspace)";

/// Shown when a tool is a solid.
pub const CONVERTED_NOTE: &str = "Solid tools are converted to mesh for this mesh target";

/// What the preview was last requested for.
#[derive(Clone, Debug, PartialEq)]
struct PreviewKey {
    target: BodyId,
    tools: Vec<BodyId>,
    params: OpParams,
    new_body: bool,
}

/// The kind of an operand body.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum Kind {
    Mesh,
    Solid,
    Implicit,
    Other,
}

fn kind(ev: &Evaluated, body: BodyId) -> Option<Kind> {
    Some(match &ev.body(body)?.geom {
        BodyGeom::Mesh(_) => Kind::Mesh,
        BodyGeom::Solid(_) => Kind::Solid,
        BodyGeom::Implicit(_) => Kind::Implicit,
        BodyGeom::Form(_) => Kind::Other,
    })
}

fn body_of(i: &SelItem) -> Option<BodyId> {
    match i.target {
        PickTarget::Body {
            body, sub: None, ..
        } => Some(body),
        _ => None,
    }
}

fn body_item(body: BodyId) -> Option<SelItem> {
    SelItem::from_target(PickTarget::Body {
        body,
        instance: 0,
        sub: None,
    })
}

/// The Combine dialog tool.
pub struct CombineTool {
    params: OpParams,
    picker: SelectTool,
    target: Vec<SelItem>,
    target_active: bool,
    tools: Vec<SelItem>,
    tools_active: bool,
    new_body: bool,
    previewed: Option<PreviewKey>,
}

impl std::fmt::Debug for CombineTool {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("CombineTool")
            .field("params", &self.params)
            .field("target", &self.target.len())
            .field("tools", &self.tools.len())
            .finish_non_exhaustive()
    }
}

impl CombineTool {
    /// The tool, taking the preselected bodies: the first mesh body is the target, the other
    /// bodies are tools. Without a preselected mesh body, the design's only mesh body is the
    /// target.
    pub fn new(init: &ToolInit<'_>) -> CombineTool {
        let ev = init.env.evaluated;
        let mut target: Vec<SelItem> = Vec::new();
        let mut tools: Vec<SelItem> = Vec::new();
        for item in init.preselection {
            let Some(b) = body_of(item) else { continue };
            if target.is_empty() && mesh_of(ev, b).is_some() {
                target.push(item.clone());
            } else {
                tools.push(item.clone());
            }
        }
        if target.is_empty() {
            let mut all = mesh_bodies(ev);
            if let (Some(only), None) = (all.next(), all.next())
                && !tools.iter().any(|t| body_of(t) == Some(only))
                && let Some(item) = body_item(only)
            {
                target.push(item);
            }
        }
        CombineTool {
            params: OpParams::new(MeshOp::Combine),
            picker: SelectTool::default(),
            target_active: target.is_empty(),
            tools_active: !target.is_empty() && tools.is_empty(),
            target,
            tools,
            new_body: false,
            previewed: None,
        }
    }

    /// The dialog state (tests read it).
    pub fn params(&self) -> &OpParams {
        &self.params
    }

    fn target_body(&self) -> Option<BodyId> {
        self.target.iter().find_map(body_of)
    }

    fn tool_bodies(&self) -> Vec<BodyId> {
        let target = self.target_body();
        let mut v: Vec<BodyId> = Vec::new();
        for b in self.tools.iter().filter_map(body_of) {
            if Some(b) != target && !v.contains(&b) {
                v.push(b);
            }
        }
        v
    }

    /// The feature the dialog describes now, or why there is none.
    fn build(&self, env: &CmdEnv<'_>) -> Result<(FeatureBox, PreviewKey), Cow<'static, str>> {
        let ev = env.evaluated;
        let target = self.target_body().ok_or("Select the target mesh body")?;
        let tools = self.tool_bodies();
        let kinds: Vec<Option<Kind>> = std::iter::once(target)
            .chain(tools.iter().copied())
            .map(|b| kind(ev, b))
            .collect();
        if kinds.contains(&Some(Kind::Implicit)) {
            return Err(IMPLICIT_NOTE.into());
        }
        match kinds[0] {
            Some(Kind::Mesh) => {}
            _ => return Err("The target must be a mesh body".into()),
        }
        if tools.is_empty() {
            return Err("Select the tool bodies".into());
        }
        if kinds[1..]
            .iter()
            .any(|k| !matches!(k, Some(Kind::Mesh | Kind::Solid)))
        {
            return Err("Tools must be mesh or solid bodies".into());
        }
        let feature = self.params.combine_feature(target, &tools, self.new_body)?;
        Ok((
            feature,
            PreviewKey {
                target,
                tools,
                params: self.params.clone(),
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

impl Tool for CombineTool {
    fn id(&self) -> CmdId {
        MeshOp::Combine.cmd()
    }

    fn label(&self) -> Cow<'static, str> {
        MeshOp::Combine.title().into()
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
        let state = cx.preview().state().clone();
        let (params, prefs, ok, health, computing, progress, implicit, solid, target_info) = {
            let env = cx.env();
            let ev = env.evaluated;
            let ok = match self.build(&env) {
                Ok(_) => Enablement::Enabled,
                Err(why) => Enablement::Disabled(why),
            };
            let health = match &state {
                PreviewState::Ready(e) => preview_health(&env, e),
                PreviewState::Failed(m) => Some(Health::Error(Diagnostic {
                    message: m.clone(),
                    cause: None,
                    entities: Vec::new(),
                })),
                _ => None,
            };
            let computing = matches!(state, PreviewState::Pending | PreviewState::Computing);
            let target = self.target_body();
            let kinds: Vec<Option<Kind>> = target
                .into_iter()
                .chain(self.tool_bodies())
                .map(|b| kind(ev, b))
                .collect();
            let implicit = kinds.contains(&Some(Kind::Implicit));
            let mesh_target = target.is_some_and(|b| kind(ev, b) == Some(Kind::Mesh));
            let solid = !implicit
                && mesh_target
                && self
                    .tool_bodies()
                    .iter()
                    .any(|&b| kind(ev, b) == Some(Kind::Solid));
            let target_info = target.and_then(|b| {
                mesh_of(ev, b)
                    .map(|m| format!("Target {}: {} faces", body_label(b), m.face_count()))
            });
            (
                env.design.params().clone(),
                env.design.meta().default_units.clone(),
                ok,
                health,
                computing,
                preview_progress(&env),
                implicit,
                solid,
                target_info,
            )
        };
        let mut cancel_preview = false;
        let action = DialogFrame::new(MeshOp::Combine.title())
            .banner(health.as_ref())
            .ok_enabled(ok)
            .show_inside(ui, |ui| {
                SelectionField::new(
                    "Target",
                    &mut self.target,
                    EntityFilter::BODY,
                    &mut self.target_active,
                )
                .limits(1, Some(1))
                .mirror(cx.selection_mut())
                .show(ui);
                SelectionField::new(
                    "Tools",
                    &mut self.tools,
                    EntityFilter::BODY,
                    &mut self.tools_active,
                )
                .limits(1, None)
                .mirror(cx.selection_mut())
                .show(ui);
                if let Some(info) = &target_info {
                    ui.weak(info);
                }
                self.params.ui(ui, &params, &prefs);
                toggle(ui, "Create new body", &mut self.new_body);
                if implicit {
                    ui.colored_label(ui.visuals().warn_fg_color, IMPLICIT_NOTE);
                } else if solid {
                    ui.colored_label(ui.visuals().warn_fg_color, CONVERTED_NOTE);
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
        if cancel_preview {
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
        EntityFilter::BODY
    }

    fn status_hint(&self) -> Option<Cow<'static, str>> {
        Some("Combine: pick the target and the tools, then OK (Enter) or Esc".into())
    }

    fn finish(&mut self, reason: FinishReason, cx: &mut ToolCx<'_>) -> Option<CommitBatch> {
        if reason != FinishReason::Commit {
            return None;
        }
        let env = cx.env();
        let (feature, _) = self.build(&env).ok()?;
        Some(CommitBatch {
            label: format!("Mesh {}", MeshOp::Combine.title()),
            commands: vec![Self::add_command(&env, feature)],
        })
    }
}
