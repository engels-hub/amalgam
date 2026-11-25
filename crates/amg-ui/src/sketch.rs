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
//! - [`shapes`] and [`create`] (UI-13): the other creation tools ([`CreateKind`]: rectangles,
//!   circles, arcs, polygon, slots, ellipse, point) on the SK-21 / SK-22 builders, with the
//!   same inference and heads-up input; [`CONSTRUCTION_CMD`] (`X`) toggles construction.
//! - [`constrain`], [`dimension`] and [`glyphs`] (UI-14): the twelve constraint tools
//!   ([`ConstraintTool`], SKETCH ▸ Constraints), Sketch Dimension (`D`; the kind follows the
//!   picks and the label placement), the SK-04 check with the "Create as driven dimension?"
//!   prompt, constraint glyph icons (hover lights up the constrained entities; click selects,
//!   Delete removes), inline dimension editing (double-click a label) and label drag.
//! - [`modify`] (UI-16): the Modify tools ([`ModifyKind`]: fillet, chamfer, trim, extend,
//!   break, offset, mirror, rectangular / circular pattern, move/copy, rotate, scale) on the S03
//!   edit operations (SK-07, SK-16, SK-17, SK-18), one undo entry per operation.
//! - [`FINISH_SKETCH_CMD`], Esc with no sketch tool running and the palette's Finish button
//!   leave the session.

pub mod constrain;
pub mod create;
pub mod dimension;
pub mod glyphs;
pub mod infer;
pub mod modify;
pub mod palette;
pub mod session;
pub mod shapes;
pub mod solver;
pub mod tools;
pub mod view;

use crate::command::{CmdAction, CmdId, CommandDef, Enablement, Enabler, Placement};
use crate::design::SKETCH_CONTEXT;
use crate::module::{GroupId, Registrar, UiModule};
use crate::tool::{Tool, ToolInit};
use crate::{app::AppCx, app::CmdEnv, timeline};

pub use constrain::{ConstrainTool, ConstraintTool};
pub use create::CreateTool;
pub use dimension::{DIMENSION_DEF, DimensionEditDialog, DimensionTool, DrivenPromptDialog};
pub use infer::{Anchor, Inference};
pub use modify::{ModifyKind, ModifyOp, ModifyOptions, ModifyTool};
pub use session::{
    DrivenOffer, Inferred, PaletteOptions, SKETCH_LAYER, SKETCH_SELECT_CMD, SessionStats,
    SketchLayer, SketchSession,
};
pub use shapes::{CreateKind, PolygonOptions};
pub use tools::{CreateSketchTool, HeadsUp, LineTool, SketchSelectTool};

inventory::submit! { UiModule { id: "sketch", register } }

/// Create Sketch (the marking menu's N wedge).
pub const CREATE_SKETCH_CMD: CmdId = CmdId("sketch.sketch");
/// Finish Sketch.
pub const FINISH_SKETCH_CMD: CmdId = CmdId("sketch.finish");
/// The Line tool.
pub const LINE_CMD: CmdId = CmdId("sketch.line");
/// Normal / construction toggle.
pub const CONSTRUCTION_CMD: CmdId = CmdId("sketch.construction");

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

/// The tool factory of a creation tool (command actions are plain function pointers).
fn start_shape(kind: CreateKind) -> fn(&ToolInit<'_>) -> Box<dyn Tool> {
    macro_rules! start {
        ($k:ident) => {
            |init| Box::new(CreateTool::new(CreateKind::$k, init))
        };
    }
    match kind {
        CreateKind::Rect2Point => start!(Rect2Point),
        CreateKind::Rect3Point => start!(Rect3Point),
        CreateKind::RectCenter => start!(RectCenter),
        CreateKind::CircleCenter => start!(CircleCenter),
        CreateKind::Circle2Point => start!(Circle2Point),
        CreateKind::Circle3Point => start!(Circle3Point),
        CreateKind::Circle2Tangent => start!(Circle2Tangent),
        CreateKind::Arc3Point => start!(Arc3Point),
        CreateKind::ArcCenter => start!(ArcCenter),
        CreateKind::ArcTangent => start!(ArcTangent),
        CreateKind::Polygon => start!(Polygon),
        CreateKind::SlotCenterToCenter => start!(SlotCenterToCenter),
        CreateKind::SlotOverall => start!(SlotOverall),
        CreateKind::SlotCenterPoint => start!(SlotCenterPoint),
        CreateKind::SlotArc => start!(SlotArc),
        CreateKind::Ellipse => start!(Ellipse),
        CreateKind::Point => start!(Point),
    }
}

/// The tool factory of a constraint tool.
fn start_constraint(tool: ConstraintTool) -> fn(&ToolInit<'_>) -> Box<dyn Tool> {
    macro_rules! start {
        ($k:ident) => {
            |init| Box::new(ConstrainTool::new(ConstraintTool::$k, init))
        };
    }
    match tool {
        ConstraintTool::HorizontalVertical => start!(HorizontalVertical),
        ConstraintTool::Coincident => start!(Coincident),
        ConstraintTool::Tangent => start!(Tangent),
        ConstraintTool::Equal => start!(Equal),
        ConstraintTool::Parallel => start!(Parallel),
        ConstraintTool::Perpendicular => start!(Perpendicular),
        ConstraintTool::Fix => start!(Fix),
        ConstraintTool::Midpoint => start!(Midpoint),
        ConstraintTool::Concentric => start!(Concentric),
        ConstraintTool::Collinear => start!(Collinear),
        ConstraintTool::Symmetric => start!(Symmetric),
        ConstraintTool::Curvature => start!(Curvature),
    }
}

/// The tool factory of a modify tool.
fn start_modify(kind: ModifyKind) -> fn(&ToolInit<'_>) -> Box<dyn Tool> {
    macro_rules! start {
        ($k:ident) => {
            |init| Box::new(ModifyTool::new(ModifyKind::$k, init))
        };
    }
    match kind {
        ModifyKind::Trim => start!(Trim),
        ModifyKind::Extend => start!(Extend),
        ModifyKind::Break => start!(Break),
        ModifyKind::Fillet => start!(Fillet),
        ModifyKind::Chamfer => start!(Chamfer),
        ModifyKind::Offset => start!(Offset),
        ModifyKind::Mirror => start!(Mirror),
        ModifyKind::RectPattern => start!(RectPattern),
        ModifyKind::CircPattern => start!(CircPattern),
        ModifyKind::Move => start!(Move),
        ModifyKind::Copy => start!(Copy),
        ModifyKind::Rotate => start!(Rotate),
        ModifyKind::Scale => start!(Scale),
    }
}

fn start_dimension(init: &ToolInit<'_>) -> Box<dyn Tool> {
    Box::new(DimensionTool::new(init))
}

fn toggle_construction(cx: &mut AppCx<'_>) {
    session::with_session(cx, |s, app| {
        if let Err(e) = s.toggle_construction(app) {
            tracing::debug!("construction toggle skipped: {e}");
        }
    });
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
    let create_group = |pinned, order| Placement::Toolbar {
        group: GroupId("design.sketch.create"),
        pinned,
        order,
    };
    for (i, kind) in (1..).zip(CreateKind::ALL) {
        let def = kind.def();
        let action = CmdAction::StartTool(start_shape(kind));
        let mut cmd = CommandDef::new(def.id.0, def.label, action)
            .with_icon(def.icon)
            .with_tooltip(def.tooltip)
            .with_keywords(def.keywords)
            .with_enabler(Enabler::When(in_sketch))
            .placed(create_group(def.pinned, i * 10));
        if let Some(shortcut) = def.shortcut {
            cmd = cmd.with_shortcut(shortcut);
        }
        r.command(cmd);
    }
    r.command(
        CommandDef::new(
            CONSTRUCTION_CMD.0,
            "Normal / Construction",
            CmdAction::Run(toggle_construction),
        )
        .with_icon(ids::CONSTRUCTION)
        .with_tooltip(
            "Toggle the selected curves between normal and construction geometry (nothing \
             selected: the style of new geometry)",
        )
        .with_keywords(&["sketch", "construction", "centreline", "reference"])
        .with_shortcut("sketch:X")
        .with_enabler(Enabler::When(in_sketch))
        .placed(create_group(false, 1000)),
    );
    {
        let def = DIMENSION_DEF;
        let mut cmd = CommandDef::new(def.id.0, def.label, CmdAction::StartTool(start_dimension))
            .with_icon(def.icon)
            .with_tooltip(def.tooltip)
            .with_keywords(def.keywords)
            .with_enabler(Enabler::When(in_sketch))
            .placed(create_group(def.pinned, 900));
        if let Some(shortcut) = def.shortcut {
            cmd = cmd.with_shortcut(shortcut);
        }
        r.command(cmd);
    }
    for (i, tool) in (1..).zip(ConstraintTool::ALL) {
        let def = tool.def();
        r.command(
            CommandDef::new(
                def.id.0,
                def.label,
                CmdAction::StartTool(start_constraint(tool)),
            )
            .with_icon(def.icon)
            .with_tooltip(def.tooltip)
            .with_keywords(def.keywords)
            .with_enabler(Enabler::When(in_sketch))
            .placed(Placement::Toolbar {
                group: GroupId("design.sketch.constraints"),
                pinned: def.pinned,
                order: i * 10,
            }),
        );
    }
    for (i, kind) in (1..).zip(ModifyKind::ALL) {
        let def = kind.def();
        let mut cmd = CommandDef::new(
            def.id.0,
            def.label,
            CmdAction::StartTool(start_modify(kind)),
        )
        .with_icon(def.icon)
        .with_tooltip(def.tooltip)
        .with_keywords(def.keywords)
        .with_enabler(Enabler::When(in_sketch))
        .placed(Placement::Toolbar {
            group: GroupId("design.sketch.modify"),
            pinned: def.pinned,
            order: i * 10,
        });
        if let Some(shortcut) = def.shortcut {
            cmd = cmd.with_shortcut(shortcut);
        }
        r.command(cmd);
    }
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
