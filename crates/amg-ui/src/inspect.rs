//! Inspection UI (lane A, S05): the Measure tool (INS-08), the Properties panel (INS-03) and
//! Section Analysis (INS-09). The INSPECT tab itself is registered by the framework (`design`).
//!
//! - [`MEASURE_CMD`] ("Measure", shortcut `I`) is in the INSPECT tab's *Inspect* group and in
//!   SOLID ▸ Inspect, and in S-key search. It starts [`measure::MeasureTool`].
//! - [`snap`]: pick → snapped, classified pick → INS-01 operand (smart regions and chains).
//! - [`props`]: the Properties panel ([`PROPERTIES_CMD`], mass properties of mesh and solid
//!   bodies, occurrences and components, with the centre-of-mass glyph).
//! - [`colors`]: "Display Component Colours" ([`COMPONENT_COLORS_CMD`], INS-10).
//! - [`SECTION_CMD`] ("Section Analysis") starts [`section_tool::SectionTool`]; [`section`] holds
//!   the per-document layer (records → RND-06 planes, caps job, outlines), the browser
//!   "Analysis" folder and the Section Statistics panel ([`SECTION_STATS_CMD`]).
//! - [`overlay`]: the RND-07 overlay (witness line, Δ, angle arc, circles), drawn through the
//!   additive `Tool::overlay` hook.

pub mod colors;
pub mod measure;
pub mod overlay;
pub mod props;
pub mod section;
pub mod section_tool;
pub mod snap;

use crate::command::{CmdAction, CmdId, CommandDef, Enabler, Placement};
use crate::module::{GroupId, Registrar, UiModule};
use crate::tool::{Tool, ToolInit};

pub use colors::{COMPONENT_COLORS_CMD, COMPONENT_COLORS_FLAG};
pub use measure::{MEASURE_JOB, MeasureSnapshot, MeasureTool, measure_snapshot};
pub use props::{PROPERTIES_CMD, PROPERTIES_JOB, PROPERTIES_PANEL};
pub use section::{SECTION_CMD, SECTION_JOB, SECTION_PANEL, SECTION_STATS_CMD, SectionLayer};
pub use section_tool::{SectionTool, section_tool_snapshot};
pub use snap::{EDGE_SNAP_PX, MeasureFilter, VERTEX_SNAP_PX};

inventory::submit! { UiModule { id: "inspect", register } }

/// Starts the Measure tool.
pub const MEASURE_CMD: CmdId = CmdId("inspect.measure");

fn start_measure(init: &ToolInit<'_>) -> Box<dyn Tool> {
    Box::new(MeasureTool::new(init))
}

fn start_section(init: &ToolInit<'_>) -> Box<dyn Tool> {
    Box::new(SectionTool::new(init))
}

/// Registers the module's commands.
pub fn register(r: &mut Registrar<'_>) {
    r.command(
        CommandDef::new(
            MEASURE_CMD.0,
            "Measure",
            CmdAction::StartTool(start_measure),
        )
        .with_icon(crate::icons::ids::MEASURE)
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
        .with_icon(crate::icons::ids::PROPERTIES)
        .with_tooltip(
            "Volume, area, mass, centre of mass and inertia of the selected bodies and \
             components, with the density of their material",
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
        .with_enabler(Enabler::When(props::enabled))
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
    r.command(
        CommandDef::new(
            SECTION_CMD.0,
            "Section Analysis",
            CmdAction::StartTool(start_section),
        )
        .with_icon(crate::icons::ids::SECTION)
        .with_tooltip(
            "Cut the view with a plane (origin plane or picked face, offset, tilt, flip): caps,              hatching and the section area; saved with the design, listed under Analysis",
        )
        .with_keywords(&[
            "inspect",
            "section",
            "cut",
            "clip",
            "cross section",
            "slice",
            "area",
            "hatch",
        ])
        .with_enabler(Enabler::When(measure::enabled))
        .placed(Placement::Toolbar {
            group: GroupId("design.inspect.inspect"),
            pinned: true,
            order: 5,
        })
        .placed(Placement::Toolbar {
            group: GroupId("design.solid.inspect"),
            pinned: false,
            order: 5,
        }),
    );
    r.command(
        CommandDef::new(
            SECTION_STATS_CMD.0,
            "Section Statistics",
            CmdAction::Run(|cx| cx.open_panel(SECTION_PANEL)),
        )
        .with_icon(crate::icons::ids::SECTION_STATS)
        .with_tooltip("Section area, perimeter, loops and extent of the visible section analyses")
        .with_keywords(&["inspect", "section", "area", "perimeter", "cross section"])
        .placed(Placement::Toolbar {
            group: GroupId("design.inspect.inspect"),
            pinned: false,
            order: 6,
        }),
    );
    props::register(r);
    colors::register(r);
    section::register(r);
}
