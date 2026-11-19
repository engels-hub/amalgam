//! Inspection UI (lane A, S05): the Measure tool (INS-08). The INSPECT tab itself is
//! registered by the framework (`design`).
//!
//! - [`MEASURE_CMD`] ("Measure", shortcut `I`) is in the INSPECT tab's *Inspect* group and in
//!   SOLID ▸ Inspect, and in S-key search. It starts [`measure::MeasureTool`].
//! - [`snap`]: pick → snapped, classified pick → INS-01 operand (smart regions and chains).
//! - [`overlay`]: the RND-07 overlay (witness line, Δ, angle arc, circles), drawn through the
//!   additive `Tool::overlay` hook.

pub mod measure;
pub mod overlay;
pub mod snap;

use crate::command::{CmdAction, CmdId, CommandDef, Enabler, Placement};
use crate::module::{GroupId, Registrar, UiModule};
use crate::tool::{Tool, ToolInit};

pub use measure::{MEASURE_JOB, MeasureSnapshot, MeasureTool, measure_snapshot};
pub use snap::{EDGE_SNAP_PX, MeasureFilter, VERTEX_SNAP_PX};

inventory::submit! { UiModule { id: "inspect", register } }

/// Starts the Measure tool.
pub const MEASURE_CMD: CmdId = CmdId("inspect.measure");

fn start_measure(init: &ToolInit<'_>) -> Box<dyn Tool> {
    Box::new(MeasureTool::new(init))
}

/// Registers the module's commands.
pub fn register(r: &mut Registrar<'_>) {
    r.command(
        CommandDef::new(
            MEASURE_CMD.0,
            "Measure",
            CmdAction::StartTool(start_measure),
        )
        .with_tooltip(
            "Measure distances, angles, lengths, areas, volumes and hole diameters (smart \
                 selection, snapping, 3-point circle)",
        )
        .with_keywords(&[
            "inspect",
            "distance",
            "angle",
            "diameter",
            "radius",
            "area",
            "volume",
            "length",
            "dimension",
            "hole",
        ])
        .with_shortcut("I")
        .with_enabler(Enabler::When(measure::enabled))
        .placed(Placement::Toolbar {
            group: GroupId("design.inspect.inspect"),
            pinned: true,
            order: 0,
        })
        .placed(Placement::Toolbar {
            group: GroupId("design.solid.inspect"),
            pinned: false,
            order: 0,
        }),
    );
}
