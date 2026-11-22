//! Mesh tools UI (lane C, S07): the MESH tab of the Design workspace (QUESTIONS #25).
//!
//! MESH-23 (Phase 1 workspace):
//! - **Toolbar and search.** One command per op ([`MeshOp`]: Repair, Remesh, Reduce, Smooth,
//!   Plane Cut, Shell, Combine (MESH-27), Separate, Reverse Normal, Erase and Fill, Wrap in
//!   *Modify*; Generate Face Groups in *Prepare*) plus [`ANALYSIS_CMD`] in *Inspect*. Every command is in a MESH
//!   toolbar group and in S-key search (labels end in `…` for commands that open a dialog).
//! - **Dialogs** ([`tool::MeshOpTool`]; Combine: [`combine::CombineTool`]) are hand-built on the framework API (UI-03 tools and
//!   previews, UI-04 selection, UI-15 widgets) and migrate to the UI-05 generic renderer in
//!   Phase 2. Each one creates the op's MESH-16/26/18/12 timeline feature (Combine: MESH-27's `mesh.combine`).
//! - **Previews** go through `Document::preview` as cancellable, latest-wins jobs; inputs above
//!   `amg_features::mesh::PREVIEW_PROXY_FACES` (2 M) are previewed on a decimated proxy. The
//!   full compute after OK is the document's recompute job (progress and Cancel in the jobs
//!   panel, UI-10).
//! - **Mesh Analysis panel** ([`analysis::AnalysisPanel`]): the MESH-02/19 report of the
//!   selected mesh body as a background job, with the defective faces on the `Error`
//!   highlight layer (RND-04) through `AppCx::set_highlight`.

pub mod analysis;
pub mod combine;
pub mod ops;
pub mod tool;

use crate::app::CmdEnv;
use crate::command::{CmdAction, CmdId, CommandDef, Enablement, Enabler, Placement};
use crate::design::DESIGN;
use crate::module::{GroupDef, GroupId, TabDef, TabId};
use crate::module::{Registrar, UiModule};
use crate::panel::{DockSlot, PanelDef};
use crate::tool::{Tool, ToolInit};

pub use analysis::{ANALYSIS_PANEL, AnalysisPanel, DefectFaces, MeshAnalysis, analyse};
pub use combine::CombineTool;
pub use ops::{MeshOp, OpParams};
pub use tool::MeshOpTool;

inventory::submit! { UiModule { id: "mesh", register } }

/// The MESH toolbar tab.
pub const MESH_TAB: TabId = TabId("mesh");
/// Opens the Mesh Analysis panel.
pub const ANALYSIS_CMD: CmdId = CmdId("mesh.analysis");

/// Every command this module registers: the ops, then the analysis.
pub fn command_ids() -> Vec<CmdId> {
    MeshOp::ALL
        .iter()
        .map(|o| o.cmd())
        .chain([ANALYSIS_CMD])
        .collect()
}

/// Enabled while the design has a mesh body.
fn has_mesh_body(env: &CmdEnv<'_>) -> Enablement {
    if analysis::mesh_bodies(env.evaluated).next().is_some() {
        Enablement::Enabled
    } else {
        Enablement::disabled("The design has no mesh body")
    }
}

macro_rules! starters {
    ($($name:ident => $op:path),* $(,)?) => {
        $(fn $name(init: &ToolInit<'_>) -> Box<dyn Tool> {
            Box::new(MeshOpTool::new($op, init))
        })*
        /// The `StartTool` factory of `op`.
        fn starter(op: MeshOp) -> fn(&ToolInit<'_>) -> Box<dyn Tool> {
            match op {
                $($op => $name,)*
                MeshOp::Combine => start_combine,
            }
        }
    };
}

/// The Combine dialog has its own tool (two body fields).
fn start_combine(init: &ToolInit<'_>) -> Box<dyn Tool> {
    Box::new(CombineTool::new(init))
}

starters! {
    start_repair => MeshOp::Repair,
    start_remesh => MeshOp::Remesh,
    start_reduce => MeshOp::Reduce,
    start_smooth => MeshOp::Smooth,
    start_plane_cut => MeshOp::PlaneCut,
    start_shell => MeshOp::Shell,
    start_separate => MeshOp::Separate,
    start_reverse_normal => MeshOp::ReverseNormal,
    start_erase_fill => MeshOp::EraseFill,
    start_wrap => MeshOp::Wrap,
    start_face_groups => MeshOp::FaceGroups,
}

/// Registers the module's workspaces, tabs, groups and panels.
pub fn register(r: &mut Registrar<'_>) {
    r.toolbar_tab(TabDef {
        id: MESH_TAB,
        workspace: DESIGN,
        label: "Mesh",
        order: 20,
        contextual: None,
    });
    for (order, (id, label)) in [
        (ops::GROUP_CREATE.0, "Create"),
        (ops::GROUP_PREPARE.0, "Prepare"),
        (ops::GROUP_MODIFY.0, "Modify"),
        (ops::GROUP_INSPECT.0, "Inspect"),
    ]
    .into_iter()
    .enumerate()
    {
        r.toolbar_group(GroupDef {
            id: GroupId(id),
            tab: MESH_TAB,
            label,
            order: order as i32 * 10,
        });
    }
    for (order, op) in (0i32..).zip(MeshOp::ALL) {
        r.command(
            CommandDef::new(op.cmd().0, op.label(), CmdAction::StartTool(starter(op)))
                .with_tooltip(op.tooltip())
                .with_keywords(op.keywords())
                .with_enabler(Enabler::When(has_mesh_body))
                .placed(Placement::Toolbar {
                    group: op.group(),
                    pinned: op.pinned(),
                    order: order * 10,
                }),
        );
    }
    r.command(
        CommandDef::new(
            ANALYSIS_CMD.0,
            "Mesh Analysis",
            CmdAction::Run(|cx| cx.open_panel(ANALYSIS_PANEL)),
        )
        .with_tooltip("Report and highlight holes, non-manifold, flipped and intersecting faces")
        .with_keywords(&["inspect", "defects", "check", "watertight", "manifold"])
        .placed(Placement::Toolbar {
            group: ops::GROUP_INSPECT,
            pinned: true,
            order: 0,
        }),
    );
    r.panel(PanelDef {
        id: ANALYSIS_PANEL,
        title: "Mesh Analysis",
        icon: crate::module::IconId::NONE,
        slot: DockSlot::Right,
        workspaces: Some(vec![DESIGN]),
        create: analysis::create_panel,
    });
}
