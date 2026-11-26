//! The position guard (additive, S06 ASM-02; S11 Interface changes 2026-11-26).
//!
//! While an assembly has moves that are not captured (`Design::has_pending_positions`), a
//! position-dependent action first asks "Capture Position / Revert / Cancel" (S06 §2). The
//! framework does not know about positions beyond that test: the timeline hands its marker
//! drags, reorders and node edits to [`guarded`], which runs them at once when nothing is
//! pending or no guard is registered, and otherwise passes them to the guard registered with
//! [`Registrar::position_guard`](crate::module::Registrar::position_guard) (`amg-ui::assembly`).
//! The guard either gives the action back (run it now) or keeps it and runs it later with
//! [`GuardedAction::run`], after the user chose. The UI never blocks on the question.
//!
//! Without a guard the action runs: `SetMarker` itself drops pending moves in the same undo
//! entry, so headless use stays deterministic.

use amg_doc::{Command, FeatureId};

use crate::app::{AppCx, CommitBatch};
use crate::command::CmdId;

/// An action that depends on the occurrence positions.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum GuardedAction {
    /// Move the timeline marker to `index` (`Command::SetMarker`).
    SetMarker {
        /// Marker index.
        index: usize,
    },
    /// Reorder a timeline node (`Command::MoveFeature`).
    MoveFeature {
        /// The node.
        id: FeatureId,
        /// Its new index.
        to: usize,
    },
    /// Edit a timeline node ([`AppCx::edit_feature`]).
    EditFeature(FeatureId),
    /// Run a command ([`AppCx::run_command`]), e.g. a tool that picks in context (Joint
    /// Origin, ASM-11).
    RunCommand(CmdId),
}

impl GuardedAction {
    /// Short user-facing description ("Move the timeline marker").
    pub fn describe(&self) -> &'static str {
        match self {
            GuardedAction::SetMarker { .. } => "Move the timeline marker",
            GuardedAction::MoveFeature { .. } => "Reorder the timeline",
            GuardedAction::EditFeature(_) => "Edit the feature",
            GuardedAction::RunCommand(_) => "Start the command",
        }
    }

    /// Does the action now, without asking the guard again.
    pub fn run(self, cx: &mut AppCx<'_>) {
        let commit = |cx: &mut AppCx<'_>, cmd: Command| {
            cx.submit(CommitBatch {
                label: cmd.label().to_owned(),
                commands: vec![cmd],
            });
        };
        match self {
            GuardedAction::SetMarker { index } => commit(cx, Command::SetMarker { index }),
            GuardedAction::MoveFeature { id, to } => commit(cx, Command::MoveFeature { id, to }),
            GuardedAction::EditFeature(id) => cx.edit_feature(id),
            GuardedAction::RunCommand(id) => cx.run_command(id),
        }
    }
}

/// A registered guard: called with an action while moves are pending. Returns the action to
/// run it now, or `None` when the guard keeps it (and runs it later with
/// [`GuardedAction::run`], or drops it on Cancel).
pub type PositionGuard = fn(&mut AppCx<'_>, GuardedAction) -> Option<GuardedAction>;

/// Runs `action` now unless moves are pending and a guard is registered; then the guard
/// decides (see the module docs).
pub fn guarded(cx: &mut AppCx<'_>, action: GuardedAction) {
    let pending = cx.env().design.has_pending_positions();
    let guard = cx.registry().position_guard();
    let action = match guard {
        Some(g) if pending => g(cx, action),
        _ => Some(action),
    };
    if let Some(a) = action {
        a.run(cx);
    }
}
