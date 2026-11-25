//! Sketch mode (S11 §13, UI-09): the sketch session, its palette, inference and the Line tool.
//!
//! - [`CREATE_SKETCH_CMD`] (`sketch.sketch`, SOLID ▸ Create and the marking menu's N wedge)
//!   starts [`tools::CreateSketchTool`]: pick an origin plane or a construction plane, and the
//!   new `sketch.sketch` feature (S03 SK-19) opens in sketch mode. Edit Sketch (browser,
//!   timeline) opens an existing one through `DocTab::sketch_edit`.
//! - [`session`]: [`SketchLayer`] (a per-document view layer) owns the [`SketchSession`]: the
//!   working-copy `SketchFeature`, the solver worker ([`solver`], one `SolverState` moved through
//!   `amg-jobs` jobs, latest wins), the DOF report and the palette options. On entry the camera
//!   looks at the plane and the SKETCH tab plus the [Sketch Palette](palette) appear.
//! - [`view`]: the RND-07 overlays (curves, points, profiles, glyphs, dimensions, DOF colours).
//! - [`infer`]: the inference engine on SK-20's `query::snap_candidates`.
//! - [`tools`]: Create Sketch, the session's select & drag tool ([`SKETCH_SELECT_CMD`]; SK-05
//!   `DragSession`, release = one `CommitDrag`), and [`LINE_CMD`] (chained lines with
//!   inference, heads-up input and tangent-arc drag-back).
//! - [`FINISH_SKETCH_CMD`], Esc with no sketch tool running and the palette's Finish button
//!   leave the session.

pub mod infer;
pub mod palette;
pub mod session;
pub mod solver;
pub mod tools;
pub mod view;

use crate::command::{CmdAction, CmdId, CommandDef, Enablement, Enabler, Placement};
use crate::design::SKETCH_CONTEXT;
use crate::module::{GroupId, Registrar, UiModule};
use crate::tool::{Tool, ToolInit};
use crate::{app::CmdEnv, timeline};

pub use infer::{Anchor, Inference};
pub use session::{
    Inferred, PaletteOptions, SKETCH_LAYER, SKETCH_SELECT_CMD, SessionStats, SketchLayer,
    SketchSession,
};
pub use tools::{CreateSketchTool, HeadsUp, LineTool, SketchSelectTool};

inventory::submit! { UiModule { id: "sketch", register } }

/// Create Sketch (the marking menu's N wedge).
pub const CREATE_SKETCH_CMD: CmdId = CmdId("sketch.sketch");
/// Finish Sketch.
pub const FINISH_SKETCH_CMD: CmdId = CmdId("sketch.finish");
/// The Line tool.
pub const LINE_CMD: CmdId = CmdId("sketch.line");

fn in_sketch(env: &CmdEnv<'_>) -> Enablement {
    if env.contexts.contains(&SKETCH_CONTEXT) {
        Enablement::Enabled
    } else {
        Enablement::Disabled("Only while a sketch is open".into())
    }
}

fn not_in_sketch(env: &CmdEnv<'_>) -> Enablement {
    if env.contexts.contains(&SKETCH_CONTEXT) {
        Enablement::Disabled("Finish the open sketch first".into())
    } else {
        Enablement::Enabled
    }
}

fn start_create(init: &ToolInit<'_>) -> Box<dyn Tool> {
    Box::new(CreateSketchTool::new(init))
}

fn start_select(init: &ToolInit<'_>) -> Box<dyn Tool> {
    Box::new(SketchSelectTool::new(init))
}

fn start_line(init: &ToolInit<'_>) -> Box<dyn Tool> {
    Box::new(LineTool::new(init))
}

/// Registers the sketch commands, the session layer and the timeline info of sketches.
pub fn register(r: &mut Registrar<'_>) {
    use crate::icons::ids;
    r.doc_layer(SKETCH_LAYER);
    r.command(
        CommandDef::new(
            CREATE_SKETCH_CMD.0,
            "Create Sketch",
            CmdAction::StartTool(start_create),
        )
        .with_icon(ids::SKETCH)
        .with_tooltip("Create a 2D sketch on an origin plane or a construction plane")
        .with_keywords(&["sketch", "2d", "draw", "plane", "profile"])
        .with_enabler(Enabler::When(not_in_sketch))
        // Not pinned: a pinned button would change the UI-01 shell snapshots (owner decision).
        .placed(Placement::Toolbar {
            group: GroupId("design.solid.create"),
            pinned: false,
            order: 0,
        })
        .placed(Placement::Toolbar {
            group: GroupId("design.surface.create"),
            pinned: false,
            order: 0,
        }),
    );
    r.command(
        CommandDef::new(
            FINISH_SKETCH_CMD.0,
            "Finish Sketch",
            CmdAction::Run(|cx| {
                if let Some(l) = cx.doc_layer::<SketchLayer>() {
                    l.request_finish();
                }
                cx.request_repaint();
            }),
        )
        .with_icon(ids::SKETCH_FINISH)
        .with_tooltip("Leave sketch mode")
        .with_keywords(&["sketch", "finish", "stop", "exit", "done"])
        .with_enabler(Enabler::When(in_sketch))
        .placed(Placement::Toolbar {
            group: GroupId("design.sketch.finish"),
            pinned: true,
            order: 0,
        }),
    );
    r.command(
        CommandDef::new(LINE_CMD.0, "Line", CmdAction::StartTool(start_line))
            .with_icon(ids::LINE)
            .with_tooltip(
                "Draw chained lines with inference; type a length, drag back from the end for a \
                 tangent arc",
            )
            .with_keywords(&["sketch", "line", "segment", "polyline", "arc"])
            .with_enabler(Enabler::When(in_sketch))
            .placed(Placement::Toolbar {
                group: GroupId("design.sketch.create"),
                pinned: true,
                order: 0,
            }),
    );
    r.command(
        CommandDef::new(
            SKETCH_SELECT_CMD.0,
            "Select Sketch Geometry",
            CmdAction::StartTool(start_select),
        )
        .with_icon(ids::SELECT)
        .with_tooltip("Select and drag sketch geometry")
        .with_keywords(&["sketch", "select", "drag", "move"])
        .with_enabler(Enabler::When(in_sketch))
        .placed(Placement::SearchOnly),
    );
    r.feature_ui(timeline::FeatureUiInfo {
        kind: crate::app::SKETCH_FEATURE_KIND,
        display_name: "Sketch",
        icon: ids::SKETCH,
        edit: timeline::EditAction::SketchMode,
    });
}
