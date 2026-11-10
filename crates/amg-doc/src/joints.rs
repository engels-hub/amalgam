//! Joint data shared with the position layers (S06, ASM-13) and the Capture Position helpers.
//!
//! ASM-04 grows this module (joint kinds, evaluated joints, the solver). ASM-13 needs only the
//! authored joint values that pending moves and capture nodes carry, and the two helpers the
//! UI uses: [`capture_command`] and [`out_of_date`].

use serde::{Deserialize, Serialize};

use crate::command::Command;
use crate::eval::Evaluated;
use crate::id::OccurrenceId;
use crate::model::{Design, DesignMode};

/// Which value of a joint a number belongs to (S06 note §5).
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum ValueSel {
    /// Rotation about the motion axis, rad.
    Rotation,
    /// Slide along the motion axis, mm.
    Slide,
    /// Slide along X of a planar joint, mm.
    SlideX,
    /// Slide along Y of a planar joint, mm.
    SlideY,
    /// Pitch of a ball joint, rad.
    Pitch,
    /// Yaw of a ball joint, rad.
    Yaw,
    /// Roll of a ball joint, rad.
    Roll,
}

/// Unwrapped joint values: the last known angles (beyond ±π) and slides of one joint. Rad for
/// rotations, mm for slides. Layered like positions (pending, capture nodes, base).
#[derive(Clone, Debug, PartialEq, Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct JointState {
    /// `(value, number)` pairs.
    pub values: Vec<(ValueSel, f64)>,
}

impl JointState {
    /// True if every number is finite.
    pub fn is_finite(&self) -> bool {
        self.values.iter().all(|(_, v)| v.is_finite())
    }
}

/// The Capture Position command for the current state: the solved placements (local
/// transforms) of every occurrence with a pending move and of every out-of-date one, and the
/// pending joint states. `None` in direct mode or when there is nothing to capture.
///
/// The command carries the values, so the crash journal replays it without a solve.
pub fn capture_command(design: &Design, eval: &Evaluated, name: Option<String>) -> Option<Command> {
    if design.meta().mode == DesignMode::Direct {
        return None;
    }
    let mut occs: Vec<OccurrenceId> = design.pending_positions().moves().map(|(o, _)| o).collect();
    occs.extend(out_of_date(design, eval));
    occs.sort_unstable();
    occs.dedup();
    let placements = eval.placements();
    let moves: Vec<_> = occs
        .into_iter()
        .filter_map(|o| placements.local(o).map(|t| (o, t)))
        .collect();
    let joint_states: Vec<_> = design
        .pending_positions()
        .joint_states()
        .map(|(f, s)| (f, s.clone()))
        .collect();
    if moves.is_empty() && joint_states.is_empty() {
        return None;
    }
    Some(Command::CapturePositions {
        moves,
        joint_states,
        name,
    })
}

/// Occurrences whose solved placement differs from [`Design::seed_transform`] by more than the
/// model tolerance (translation by the linear tolerance, the linear part by the angular
/// one): the "positions out of date" badge. Ascending ids.
pub fn out_of_date(design: &Design, eval: &Evaluated) -> Vec<OccurrenceId> {
    let tol = &design.meta().tolerance;
    let placements = eval.placements();
    let table = design.capture_table(design.timeline().marker());
    design
        .occurrences()
        .filter_map(|(o, _)| {
            let seed = design.seed_with(&table, o)?;
            let solved = placements.local(o)?;
            let dt = (seed.translation - solved.translation).abs().max_element();
            let dm = (seed.matrix3.to_cols_array())
                .iter()
                .zip(solved.matrix3.to_cols_array())
                .map(|(a, b)| (a - b).abs())
                .fold(0.0, f64::max);
            (dt > tol.linear || dm > tol.angular).then_some(o)
        })
        .collect()
}
