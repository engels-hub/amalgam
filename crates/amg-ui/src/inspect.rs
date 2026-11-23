//! Inspection UI (lane A, S05): the Measure tool (INS-08) and the Properties panel (INS-03). The INSPECT tab itself is
//! registered by the framework (`design`).
//!
//! - [`MEASURE_CMD`] ("Measure", shortcut `I`) is in the INSPECT tab's *Inspect* group and in
//!   SOLID ▸ Inspect, and in S-key search. It starts [`measure::MeasureTool`].
//! - [`snap`]: pick → snapped, classified pick → INS-01 operand (smart regions and chains).
//! - [`props`]: the Properties panel ([`PROPERTIES_CMD`], mass properties of mesh bodies with
//!   the centre-of-mass glyph).
//! - [`overlay`]: the RND-07 overlay (witness line, Δ, angle arc, circles), drawn through the
//!   additive `Tool::overlay` hook.

pub mod measure;
pub mod overlay;
pub mod props;
pub mod snap;

use crate::command::{CmdAction, CmdId, CommandDef, Enabler, Placement};
use crate::module::{GroupId, Registrar, UiModule};
use crate::tool::{Tool, ToolInit};

pub use measure::{MEASURE_JOB, MeasureSnapshot, MeasureTool, measure_snapshot};
pub use props::{PROPERTIES_CMD, PROPERTIES_JOB, PROPERTIES_PANEL};
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
    r.command(
        CommandDef::new(
            PROPERTIES_CMD.0,
            "Properties",
            CmdAction::Run(|cx| cx.open_panel(PROPERTIES_PANEL)),
        )
        .with_tooltip(
            "Volume, area, mass, centre of mass and inertia of the selected mesh bodies, with              the density of their material",
        )
        .with_keywords(&[
            "inspect",
            "mass",
            "weight",
            "volume",
            "density",
            "inertia",
            "centre of mass",
            "center of gravity",
            "physical properties",
        ])
        .with_enabler(Enabler::When(measure::enabled))
        .placed(Placement::Toolbar {
            group: GroupId("design.inspect.inspect"),
            pinned: true,
            order: 10,
        })
        .placed(Placement::Toolbar {
            group: GroupId("design.solid.inspect"),
            pinned: false,
            order: 10,
        }),
    );
    props::register(r);
}
