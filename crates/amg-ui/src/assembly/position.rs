//! The Position panel and the Capture / Revert / Cancel prompt (S06 §2, §7, ASM-02).
//!
//! - **Position panel:** while the design has pending moves (History mode) a small panel at
//!   the top of the window offers *Capture Position* (one `CapturePositions`: a timeline node)
//!   and *Revert* (one `RevertPositions`: back to the captured positions). It goes away as
//!   soon as nothing is pending, and hides while a prompt is open. The document layer opens
//!   it.
//! - **Prompt:** [`guard`] is registered as the S11 position guard ([`crate::guard`]). A
//!   position-dependent action while moves are pending (timeline marker, reorder, edit; a
//!   tool that calls [`guard_tool_start`], such as Joint Origin) opens the prompt instead of
//!   running. *Capture Position* and *Revert* do what the panel does and then run the action
//!   once nothing is pending; *Cancel* drops it and leaves the design as it was. One prompt
//!   at a time: an action asked for while one is open is dropped.

use std::borrow::Cow;
use std::cell::Cell;
use std::rc::Rc;

use amg_doc::{Command, DesignMode};

use super::actions;
use super::state::AssemblyLayer;
use crate::app::{AppCx, CommitBatch};
use crate::command::CmdId;
use crate::dialog::AppDialog;
use crate::guard::GuardedAction;
use crate::tool::{FinishReason, ToolCx};

/// Labels of the panel and prompt buttons and the prompt's title.
pub mod position_labels {
    /// Capture Position button.
    pub const CAPTURE: &str = "Capture Position";
    /// Revert button.
    pub const REVERT: &str = "Revert";
    /// Cancel button (prompt).
    pub const CANCEL: &str = "Cancel";
    /// The panel's title.
    pub const PANEL: &str = "Position";
    /// The prompt's title.
    pub const PROMPT: &str = "Positions not captured";
}

/// Frames a prompt waits for its Capture / Revert to apply before it gives up.
const WAIT_FRAMES: u32 = 600;

fn pending(cx: &AppCx<'_>) -> bool {
    let d = cx.doc_tab().document.design();
    d.meta().mode == DesignMode::History && d.has_pending_positions()
}

/// Revert: one `RevertPositions`.
pub(super) fn revert(cx: &mut AppCx<'_>) {
    if pending(cx) {
        cx.submit(CommitBatch {
            label: "Revert".into(),
            commands: vec![Command::RevertPositions],
        });
    }
}

/// The Position panel (see the module docs).
#[derive(Debug)]
pub struct PositionPanel {
    open: Rc<Cell<bool>>,
    /// A prompt is open: it offers the same buttons, so the panel steps aside.
    prompt: Rc<Cell<bool>>,
}

impl PositionPanel {
    pub(super) fn new(open: Rc<Cell<bool>>, prompt: Rc<Cell<bool>>) -> PositionPanel {
        PositionPanel { open, prompt }
    }
}

impl AppDialog for PositionPanel {
    fn title(&self) -> Cow<'static, str> {
        position_labels::PANEL.into()
    }

    fn show(&mut self, ctx: &egui::Context, cx: &mut AppCx<'_>) -> bool {
        if !pending(cx) {
            self.open.set(false);
            return false;
        }
        if self.prompt.get() {
            return true;
        }
        let (mut capture, mut revert_now) = (false, false);
        egui::Window::new(position_labels::PANEL)
            .id(egui::Id::new("amg_assembly_position_panel"))
            .anchor(egui::Align2::CENTER_TOP, [0.0, 72.0])
            .collapsible(false)
            .resizable(false)
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    capture = ui
                        .button(position_labels::CAPTURE)
                        .on_hover_text("Keep the moved positions: adds a timeline node")
                        .clicked();
                    revert_now = ui
                        .button(position_labels::REVERT)
                        .on_hover_text("Put the moved components back")
                        .clicked();
                });
            });
        if capture {
            actions::try_capture(cx);
        } else if revert_now {
            revert(cx);
        }
        true
    }

    fn as_any(&self) -> &dyn std::any::Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
        self
    }
}

/// What the user chose in the prompt.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum Chosen {
    Capture,
    Revert,
}

/// The Capture / Revert / Cancel prompt (see the module docs).
#[derive(Debug)]
pub struct PositionPrompt {
    action: GuardedAction,
    open: Rc<Cell<bool>>,
    /// After Capture / Revert: the marker before it and the frames waited.
    waiting: Option<(Chosen, usize, u32)>,
}

impl PositionPrompt {
    /// The action that waits for the answer.
    pub fn action(&self) -> &GuardedAction {
        &self.action
    }

    /// The action adjusted for a capture node inserted at `before` (the marker then).
    fn adjusted(&self, chosen: Chosen, before: usize, now: usize) -> GuardedAction {
        let shift = if chosen == Chosen::Capture {
            now.saturating_sub(before)
        } else {
            0
        };
        match self.action.clone() {
            GuardedAction::SetMarker { index } if index > before => GuardedAction::SetMarker {
                index: index + shift,
            },
            GuardedAction::MoveFeature { id, to } if to > before => {
                GuardedAction::MoveFeature { id, to: to + shift }
            }
            other => other,
        }
    }

    fn close(&self) -> bool {
        self.open.set(false);
        false
    }
}

impl AppDialog for PositionPrompt {
    fn title(&self) -> Cow<'static, str> {
        position_labels::PROMPT.into()
    }

    fn show(&mut self, ctx: &egui::Context, cx: &mut AppCx<'_>) -> bool {
        if let Some((chosen, before, frames)) = self.waiting {
            if !pending(cx) {
                let now = cx.doc_tab().document.design().timeline().marker();
                self.adjusted(chosen, before, now).run(cx);
                return self.close();
            }
            if frames >= WAIT_FRAMES {
                tracing::warn!("position prompt: the positions never cleared; action dropped");
                return self.close();
            }
            self.waiting = Some((chosen, before, frames + 1));
            cx.request_repaint();
            return true;
        }
        if !pending(cx) {
            // Captured or reverted elsewhere (panel, undo): nothing to ask any more.
            self.action.clone().run(cx);
            return self.close();
        }
        let mut choice = None;
        let what = self.action.describe();
        egui::Window::new(position_labels::PROMPT)
            .id(egui::Id::new("amg_assembly_position_prompt"))
            .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
            .collapsible(false)
            .resizable(false)
            .show(ctx, |ui| {
                ui.label(format!(
                    "Some components were moved and their positions are not captured. {what} \
                     first?"
                ));
                ui.horizontal(|ui| {
                    if ui.button(position_labels::CAPTURE).clicked() {
                        choice = Some(Some(Chosen::Capture));
                    }
                    if ui.button(position_labels::REVERT).clicked() {
                        choice = Some(Some(Chosen::Revert));
                    }
                    if ui.button(position_labels::CANCEL).clicked() {
                        choice = Some(None);
                    }
                });
            });
        let marker = cx.doc_tab().document.design().timeline().marker();
        match choice {
            Some(Some(Chosen::Capture)) => {
                if actions::try_capture(cx) {
                    self.waiting = Some((Chosen::Capture, marker, 0));
                }
                cx.request_repaint();
                true
            }
            Some(Some(Chosen::Revert)) => {
                revert(cx);
                self.waiting = Some((Chosen::Revert, marker, 0));
                cx.request_repaint();
                true
            }
            Some(None) => self.close(),
            None => true,
        }
    }

    fn as_any(&self) -> &dyn std::any::Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
        self
    }
}

/// The position guard (registered with `Registrar::position_guard`): opens the prompt for
/// `action` while moves are pending.
pub(super) fn guard(cx: &mut AppCx<'_>, action: GuardedAction) -> Option<GuardedAction> {
    if !pending(cx) {
        return Some(action);
    }
    let Some(layer) = cx.doc_layer::<AssemblyLayer>() else {
        return Some(action);
    };
    if layer.prompt_open.get() {
        return None;
    }
    let open = Rc::clone(&layer.prompt_open);
    open.set(true);
    cx.open_dialog(Box::new(PositionPrompt {
        action,
        open,
        waiting: None,
    }));
    cx.request_repaint();
    None
}

/// For position-dependent tools (Joint Origin, ASM-11), at the top of `Tool::start`: while
/// moves are pending this opens the prompt, finishes the tool (`Cancel`) and returns true; the
/// prompt runs `command` again after Capture Position or Revert. Returns false (go on) when
/// nothing is pending.
pub fn guard_tool_start(cx: &mut ToolCx<'_>, command: CmdId) -> bool {
    if guard(cx.app(), GuardedAction::RunCommand(command)).is_some() {
        return false;
    }
    cx.request_finish(FinishReason::Cancel);
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    fn prompt(action: GuardedAction) -> PositionPrompt {
        PositionPrompt {
            action,
            open: Rc::new(Cell::new(true)),
            waiting: None,
        }
    }

    #[test]
    fn a_capture_node_at_the_marker_shifts_later_targets() {
        // Marker at 3; Capture inserts a node there (marker 4).
        let p = prompt(GuardedAction::SetMarker { index: 5 });
        assert_eq!(
            p.adjusted(Chosen::Capture, 3, 4),
            GuardedAction::SetMarker { index: 6 }
        );
        assert_eq!(
            p.adjusted(Chosen::Revert, 3, 3),
            GuardedAction::SetMarker { index: 5 }
        );
        let back = prompt(GuardedAction::SetMarker { index: 1 });
        assert_eq!(
            back.adjusted(Chosen::Capture, 3, 4),
            GuardedAction::SetMarker { index: 1 }
        );
        let id = amg_doc::FeatureId(7);
        let mv = prompt(GuardedAction::MoveFeature { id, to: 4 });
        assert_eq!(
            mv.adjusted(Chosen::Capture, 3, 4),
            GuardedAction::MoveFeature { id, to: 5 }
        );
        let edit = prompt(GuardedAction::EditFeature(id));
        assert_eq!(
            edit.adjusted(Chosen::Capture, 3, 4),
            GuardedAction::EditFeature(id)
        );
    }
}
