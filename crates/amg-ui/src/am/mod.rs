//! Additive manufacturing UI (lane G, S16): the Manufacture workspace.
//!
//! AM-11 (Phase 1 workspace):
//! - **Workspace and toolbar.** The *Manufacture* workspace with the ADDITIVE tab: groups
//!   *Setup* (New Setup, Add Bodies, Remove), *Place* (Place on Plate, Drop, Move/Rotate,
//!   Centre), *Analyse* (Overhang Analysis and Overhang Settings since AM-12; Auto-Orient and
//!   Arrange registered and disabled until AM-04 / AM-05) and *Output* (Send to Slicer, Export 3MF, Manufacture Settings).
//!   Every command is also in S-key search. Every edit of a setup is one `Command::SetExt` (or
//!   `AddExt` for a new setup) in one [`CommitBatch`], so it is one undo entry.
//! - **Setup browser** ([`browser::SetupBrowser`]): Print Setups → setup (with the machine) →
//!   plates → items, in the browser of every workspace. Double-click (or *Activate*) makes a
//!   setup the one the workspace works on; without a choice the newest setup is active.
//! - **Plate scene** ([`layer::AmLayer`], a document layer): while the Manufacture workspace is
//!   active, the plate, its 10 mm grid, the build-volume edges and the keep-out areas are
//!   RND-07 overlays, and the items of the active plate are drawn at their effective transforms
//!   through the scene's per-body view overrides ([`crate::scene::BodyViews`]); all other bodies
//!   are hidden. Effective transforms and fit checks run as a latest-wins job per change, so a
//!   1 M-triangle item never costs the UI thread a vertex pass. Items that do not fit are
//!   tinted red with a message ("3.2 mm too tall") next to them and in the Plate panel.
//! - **Dialogs** are hand-built on the framework API (UI-05 comes in Phase 2): the machine
//!   picker of New Setup ([`dialogs::NewSetupDialog`]: built-in library plus the user machine
//!   folder), Add Bodies, Move/Rotate (numeric x, y, elevation, rotate Z, tilt X/Y with a live
//!   preview) and Manufacture Settings ([`settings::SettingsDialog`]: slicer path, hand-off
//!   folder, user machine folder).
//! - **Place on Plate** ([`tools::PlaceOnPlateTool`]) picks a face; the face-down rotation runs
//!   as a job. **Send to Slicer** and **Export 3MF** run AM-06's export (and launch) as
//!   `amg-jobs` jobs ([`output`]).
//!
//! AM-12 ([`overhang`]): the Overhang Analysis toggle colours the items with the RND-08
//! overhang shader (uniforms from AM-02 `shader_params`, every frame), runs AM-02
//! `analyze_overhang` as coarse (while dragging) and fine (on release) jobs with
//! cancel-and-supersede, keeps the fine per-face class buffer per item, marks the island
//! vertices and shows the item / plate metrics in the Plate panel. Overhang Settings edits α,
//! the contact tolerance and the support resolution of the setup with a live preview.

pub mod browser;
pub mod dialogs;
pub mod layer;
pub mod output;
pub mod overhang;
pub mod settings;
pub mod tools;

use std::borrow::Cow;
use std::marker::PhantomData;
use std::sync::Arc;

use amg_am::AmError;
use amg_am::machine::ProcessParams;
use amg_am::place::{Placement as AmPlacement, ZMode, centre_on_plate};
use amg_doc::{BodyGeom, BodyId, Design, Evaluated, ExtKey};
use amg_features::am::{ItemId, PlateItem, PrintSetup, setups};
use amg_jobs::{Job, JobCtx};
use amg_mesh::IndexedMesh;

use crate::app::{AppCx, CmdEnv, CommitBatch};
use crate::command::{CmdAction, CmdId, CommandDef, Enablement, Enabler, Placement};
use crate::module::{GroupDef, GroupId, Registrar, UiModule};
use crate::module::{PanelId, TabDef, TabId, WorkspaceDef, WorkspaceId};
use crate::notify::Notification;
use crate::panel::{DockSlot, PanelDef};
use crate::tool::{Tool, ToolInit};
use amg_doc::EntityRef;

pub use browser::SetupBrowser;
pub use dialogs::{AddBodiesDialog, MoveRotateDialog, NewSetupDialog, OverhangSettingsDialog};
pub use layer::{AmLayer, ItemStatus, fit_message};
pub use settings::{AmSettings, SettingsDialog};
pub use tools::PlaceOnPlateTool;

inventory::submit! { UiModule { id: "am", register } }

/// The Manufacture workspace.
pub const AM: WorkspaceId = WorkspaceId("am");
/// The ADDITIVE toolbar tab.
pub const AM_TAB: TabId = TabId("am.additive");
/// The Plate panel (setup, machine, items and their fit).
pub const PLATE_PANEL: PanelId = PanelId("am.plate");

/// Toolbar group *Setup*.
pub const GROUP_SETUP: GroupId = GroupId("am.additive.setup");
/// Toolbar group *Place*.
pub const GROUP_PLACE: GroupId = GroupId("am.additive.place");
/// Toolbar group *Analyse*.
pub const GROUP_ANALYSE: GroupId = GroupId("am.additive.analyse");
/// Toolbar group *Output*.
pub const GROUP_OUTPUT: GroupId = GroupId("am.additive.output");

/// New Setup (machine picker).
pub const NEW_SETUP: CmdId = CmdId("am.new_setup");
/// Add Bodies to the active plate.
pub const ADD_BODIES: CmdId = CmdId("am.add_bodies");
/// Remove the selected items from the active plate.
pub const REMOVE_ITEMS: CmdId = CmdId("am.remove_items");
/// Place on Plate: pick a face that goes face down.
pub const PLACE_FACE_DOWN: CmdId = CmdId("am.place_face_down");
/// Drop the items onto the plate.
pub const DROP: CmdId = CmdId("am.drop");
/// Move/Rotate dialog.
pub const MOVE_ROTATE: CmdId = CmdId("am.move_rotate");
/// Centre the items on the plate.
pub const CENTRE: CmdId = CmdId("am.centre");
/// Overhang overlay (AM-12).
pub const OVERHANG_TOGGLE: CmdId = CmdId("am.overhang_toggle");
/// Overhang Settings dialog: α, contact tolerance, support resolution (AM-12).
pub const OVERHANG_SETTINGS: CmdId = CmdId("am.overhang_settings");
/// Auto-Orient (AM-04).
pub const AUTO_ORIENT: CmdId = CmdId("am.auto_orient");
/// Arrange (AM-05).
pub const ARRANGE: CmdId = CmdId("am.arrange");
/// Send the active plate to the slicer.
pub const SEND_TO_SLICER: CmdId = CmdId("am.send_to_slicer");
/// Export the active plate as 3MF.
pub const EXPORT_3MF: CmdId = CmdId("am.export_3mf");
/// Manufacture settings (slicer path, hand-off folder, machine folder).
pub const SETTINGS: CmdId = CmdId("am.settings");

/// Every command this module registers, in toolbar order.
pub fn command_ids() -> Vec<CmdId> {
    commands().iter().map(|c| c.id).collect()
}

/// Linear RGBA of items that do not fit the build volume.
pub const OUT_OF_VOLUME_TINT: [f32; 4] = [0.85, 0.12, 0.08, 1.0];

// ---- shared helpers --------------------------------------------------------------------------

/// The setup the workspace works on: `chosen` if it still exists and decodes, else the newest
/// setup that decodes.
pub fn active_setup(design: &Design, chosen: Option<&ExtKey>) -> Option<(ExtKey, PrintSetup)> {
    let all = setups(design);
    if let Some(c) = chosen
        && let Some((k, Ok(s))) = all.iter().find(|(k, _)| k == c)
    {
        return Some((k.clone(), s.clone()));
    }
    all.into_iter()
        .rev()
        .find_map(|(k, s)| s.ok().map(|s| (k, s)))
}

/// The active setup of the document behind `cx` (the layer's choice, else the newest).
pub(crate) fn current_setup(cx: &mut AppCx<'_>) -> Option<(ExtKey, PrintSetup)> {
    let chosen = cx.doc_layer::<AmLayer>().and_then(|l| l.active.clone());
    active_setup(cx.env().design, chosen.as_ref())
}

/// The mesh of `body`, if it is a mesh body.
pub fn mesh_of(ev: &Evaluated, body: BodyId) -> Option<Arc<IndexedMesh>> {
    match &ev.body(body)?.geom {
        BodyGeom::Mesh(m) => Some(Arc::clone(m)),
        _ => None,
    }
}

/// The items of the active plate of `s`.
pub fn active_items(s: &PrintSetup) -> &[PlateItem] {
    s.plates
        .iter()
        .find(|p| p.id == s.active_plate)
        .map_or(&[], |p| p.items.as_slice())
}

/// Bodies of the selection, in selection order.
pub(crate) fn selected_bodies(cx: &AppCx<'_>) -> Vec<BodyId> {
    let mut out = Vec::new();
    for item in cx.selection().items() {
        let b = match &item.entity {
            EntityRef::Body(b) => Some(*b),
            _ => item.target.body(),
        };
        if let Some(b) = b
            && !out.contains(&b)
        {
            out.push(b);
        }
    }
    out
}

/// Items of the active plate whose body is selected; with `all_if_none`, every item of the
/// plate when none is selected.
pub(crate) fn targeted_items(cx: &AppCx<'_>, s: &PrintSetup, all_if_none: bool) -> Vec<ItemId> {
    let sel = selected_bodies(cx);
    let items = active_items(s);
    let picked: Vec<ItemId> = items
        .iter()
        .filter(|i| sel.contains(&i.body))
        .map(|i| i.id)
        .collect();
    if picked.is_empty() && all_if_none {
        items.iter().map(|i| i.id).collect()
    } else {
        picked
    }
}

/// Elevation of a freshly dropped item: the resin default elevation, else 0 (on the plate).
pub fn drop_elevation(s: &PrintSetup) -> f64 {
    match &s.machine.process {
        ProcessParams::Resin(r) => r.default_elevation.max(0.0),
        _ => 0.0,
    }
}

/// Placement of a body newly added to `s`: no rotation, at the bed centre, dropped.
pub fn initial_placement(s: &PrintSetup) -> AmPlacement {
    AmPlacement {
        rotation: amg_core::glam::DQuat::IDENTITY,
        xy: s.machine.volume.centre(),
        z: ZMode::OnPlate {
            elevation: drop_elevation(s),
        },
    }
}

/// Submits `s` as the new record under `key` (one undo entry labelled `label`).
pub(crate) fn submit_setup(cx: &mut AppCx<'_>, label: &str, key: ExtKey, s: &PrintSetup) {
    cx.submit(CommitBatch {
        label: label.to_owned(),
        commands: vec![s.command(key)],
    });
}

/// Runs `f` on the active setup and submits it if `f` returns true; tells the user when there
/// is no setup.
fn edit_active(
    cx: &mut AppCx<'_>,
    label: &str,
    f: impl FnOnce(&AppCx<'_>, &mut PrintSetup) -> bool,
) {
    let Some((key, mut s)) = current_setup(cx) else {
        cx.notify(Notification::info(
            label,
            "Create a print setup first (New Setup).",
        ));
        return;
    };
    if f(cx, &mut s) {
        submit_setup(cx, label, key, &s);
    }
}

/// A job from a closure, for [`AppCx::spawn_tracked`] (which needs a [`Job`] type).
pub(crate) struct FnJob<T, F> {
    label: Cow<'static, str>,
    f: F,
    _out: PhantomData<fn() -> T>,
}

impl<T, F> FnJob<T, F> {
    pub(crate) fn new(label: impl Into<Cow<'static, str>>, f: F) -> Self {
        FnJob {
            label: label.into(),
            f,
            _out: PhantomData,
        }
    }
}

impl<T, F> Job for FnJob<T, F>
where
    T: Send + 'static,
    F: FnOnce(&JobCtx) -> Result<T, AmError> + Send + 'static,
{
    type Output = T;
    type Error = AmError;

    fn label(&self) -> Cow<'static, str> {
        self.label.clone()
    }

    fn run(self, ctx: &JobCtx) -> Result<T, AmError> {
        (self.f)(ctx)
    }
}

// ---- command actions -------------------------------------------------------------------------

fn has_setup(env: &CmdEnv<'_>) -> Enablement {
    if active_setup(env.design, None).is_some() {
        Enablement::Enabled
    } else {
        Enablement::disabled("Create a print setup first (New Setup)")
    }
}

fn has_mesh_body(env: &CmdEnv<'_>) -> Enablement {
    if env
        .evaluated
        .bodies()
        .any(|(_, b)| matches!(b.geom, BodyGeom::Mesh(_)))
    {
        Enablement::Enabled
    } else {
        Enablement::disabled("The design has no mesh body (insert or convert one first)")
    }
}

fn has_setup_and_mesh(env: &CmdEnv<'_>) -> Enablement {
    match has_setup(env) {
        Enablement::Enabled => has_mesh_body(env),
        d => d,
    }
}

fn has_items(env: &CmdEnv<'_>) -> Enablement {
    match active_setup(env.design, None) {
        None => Enablement::disabled("Create a print setup first (New Setup)"),
        Some((_, s)) if s.plates.iter().all(|p| p.items.is_empty()) => {
            Enablement::disabled("The setup has no items (Add Bodies)")
        }
        Some(_) => Enablement::Enabled,
    }
}

fn later_orient(_: &CmdEnv<'_>) -> Enablement {
    Enablement::disabled("Arrives with AM-04 (orientation)")
}

fn later_arrange(_: &CmdEnv<'_>) -> Enablement {
    Enablement::disabled("Arrives with AM-05 (arrange)")
}

fn run_new_setup(cx: &mut AppCx<'_>) {
    let d = NewSetupDialog::new(cx);
    cx.open_dialog(Box::new(d));
}

fn run_add_bodies(cx: &mut AppCx<'_>) {
    let Some((key, s)) = current_setup(cx) else {
        cx.notify(Notification::info(
            "Add Bodies",
            "Create a print setup first (New Setup).",
        ));
        return;
    };
    let on_plate: Vec<BodyId> = active_items(&s).iter().map(|i| i.body).collect();
    let ev = Arc::clone(cx.env().evaluated);
    let picked: Vec<BodyId> = selected_bodies(cx)
        .into_iter()
        .filter(|b| !on_plate.contains(b) && mesh_of(&ev, *b).is_some())
        .collect();
    if picked.is_empty() {
        let d = AddBodiesDialog::new(cx, key, &s);
        cx.open_dialog(Box::new(d));
        return;
    }
    add_bodies(cx, key, s, &picked);
}

/// Adds `bodies` to the active plate of `s` (one undo entry).
pub(crate) fn add_bodies(cx: &mut AppCx<'_>, key: ExtKey, mut s: PrintSetup, bodies: &[BodyId]) {
    let place = initial_placement(&s);
    let plate = s.active_plate;
    for b in bodies {
        if active_items(&s).iter().any(|i| i.body == *b) {
            continue;
        }
        s.add_body(plate, *b, place);
    }
    submit_setup(cx, "Add Bodies", key, &s);
}

fn run_remove(cx: &mut AppCx<'_>) {
    edit_active(cx, "Remove from Plate", |cx, s| {
        let ids = targeted_items(cx, s, false);
        if ids.is_empty() {
            return false;
        }
        for id in ids {
            s.remove_item(id);
        }
        true
    });
}

fn start_place(init: &ToolInit<'_>) -> Box<dyn Tool> {
    Box::new(PlaceOnPlateTool::new(init))
}

fn run_drop(cx: &mut AppCx<'_>) {
    edit_active(cx, "Drop to Plate", |cx, s| {
        let ids = targeted_items(cx, s, true);
        let elevation = drop_elevation(s);
        for id in &ids {
            if let Some(it) = s.item_mut(*id) {
                it.placement.z = ZMode::OnPlate { elevation };
            }
        }
        !ids.is_empty()
    });
}

fn run_centre(cx: &mut AppCx<'_>) {
    edit_active(cx, "Centre on Plate", |cx, s| {
        let ids = targeted_items(cx, s, true);
        let volume = s.machine.volume.clone();
        for id in &ids {
            if let Some(it) = s.item_mut(*id) {
                it.placement = centre_on_plate(&it.placement, &volume);
            }
        }
        !ids.is_empty()
    });
}

fn run_move_rotate(cx: &mut AppCx<'_>) {
    match MoveRotateDialog::new(cx) {
        Ok(d) => cx.open_dialog(Box::new(d)),
        Err(why) => cx.notify(Notification::info("Move/Rotate", why)),
    }
}

fn overhang_on(_: &CmdEnv<'_>) -> bool {
    overhang::overlay_enabled()
}

fn set_overhang(cx: &mut AppCx<'_>, on: bool) {
    overhang::set_overlay_enabled(on);
    if on {
        cx.open_panel(PLATE_PANEL);
    }
}

fn run_overhang_settings(cx: &mut AppCx<'_>) {
    match OverhangSettingsDialog::new(cx) {
        Ok(d) => cx.open_dialog(Box::new(d)),
        Err(why) => cx.notify(Notification::info("Overhang Settings", why)),
    }
}

fn run_later(cx: &mut AppCx<'_>) {
    cx.notify(Notification::info(
        "Not available yet",
        "This command arrives with a later work package.",
    ));
}

fn run_send(cx: &mut AppCx<'_>) {
    output::send_to_slicer(cx);
}

fn run_export(cx: &mut AppCx<'_>) {
    output::export_3mf(cx);
}

fn run_settings(cx: &mut AppCx<'_>) {
    cx.open_dialog(Box::new(SettingsDialog::new()));
}

fn open_panel(cx: &mut AppCx<'_>) {
    cx.open_panel(PLATE_PANEL);
}

/// One command of the module.
struct Cmd {
    id: CmdId,
    label: &'static str,
    tooltip: &'static str,
    keywords: &'static [&'static str],
    group: GroupId,
    pinned: bool,
    action: CmdAction,
    enabler: Enabler,
}

fn commands() -> Vec<Cmd> {
    vec![
        Cmd {
            id: NEW_SETUP,
            label: "New Setup…",
            tooltip: "Create a print setup on a machine from the library or your machine folder",
            keywords: &[
                "print",
                "machine",
                "printer",
                "setup",
                "additive",
                "manufacture",
            ],
            group: GROUP_SETUP,
            pinned: true,
            action: CmdAction::Run(run_new_setup),
            enabler: Enabler::Always,
        },
        Cmd {
            id: ADD_BODIES,
            label: "Add Bodies",
            tooltip: "Add the selected mesh bodies to the active plate (no selection: choose them)",
            keywords: &["plate", "item", "print", "add"],
            group: GROUP_SETUP,
            pinned: true,
            action: CmdAction::Run(run_add_bodies),
            enabler: Enabler::When(has_setup_and_mesh),
        },
        Cmd {
            id: REMOVE_ITEMS,
            label: "Remove from Plate",
            tooltip: "Remove the selected items from the active plate (the bodies stay in the design)",
            keywords: &["plate", "item", "delete", "remove"],
            group: GROUP_SETUP,
            pinned: false,
            action: CmdAction::Run(run_remove),
            enabler: Enabler::When(has_items),
        },
        Cmd {
            id: PLACE_FACE_DOWN,
            label: "Place on Plate",
            tooltip: "Pick a face of an item; it is turned to lie face down on the plate",
            keywords: &["face down", "lay flat", "orient", "plate", "place"],
            group: GROUP_PLACE,
            pinned: true,
            action: CmdAction::StartTool(start_place),
            enabler: Enabler::When(has_items),
        },
        Cmd {
            id: DROP,
            label: "Drop to Plate",
            tooltip: "Lower the selected items (or all) onto the plate",
            keywords: &["plate", "z", "lower", "drop"],
            group: GROUP_PLACE,
            pinned: true,
            action: CmdAction::Run(run_drop),
            enabler: Enabler::When(has_items),
        },
        Cmd {
            id: MOVE_ROTATE,
            label: "Move/Rotate…",
            tooltip: "Move and rotate an item numerically (x, y, elevation, rotate Z, tilt)",
            keywords: &["move", "rotate", "translate", "position", "tilt", "angle"],
            group: GROUP_PLACE,
            pinned: true,
            action: CmdAction::Run(run_move_rotate),
            enabler: Enabler::When(has_items),
        },
        Cmd {
            id: CENTRE,
            label: "Centre on Plate",
            tooltip: "Move the selected items (or all) to the centre of the bed",
            keywords: &["center", "centre", "middle", "plate"],
            group: GROUP_PLACE,
            pinned: false,
            action: CmdAction::Run(run_centre),
            enabler: Enabler::When(has_items),
        },
        Cmd {
            id: OVERHANG_TOGGLE,
            label: "Overhang Analysis",
            tooltip: "Colour overhangs on the plate and show support estimates and islands",
            keywords: &["overhang", "support", "analysis", "islands"],
            group: GROUP_ANALYSE,
            pinned: true,
            action: CmdAction::Toggle {
                get: overhang_on,
                set: set_overhang,
            },
            enabler: Enabler::When(has_items),
        },
        Cmd {
            id: OVERHANG_SETTINGS,
            label: "Overhang Settings…",
            tooltip: "Critical angle, contact tolerance and support-estimate resolution of the setup",
            keywords: &[
                "critical angle",
                "alpha",
                "contact tolerance",
                "support cell",
            ],
            group: GROUP_ANALYSE,
            pinned: false,
            action: CmdAction::Run(run_overhang_settings),
            enabler: Enabler::When(has_setup),
        },
        Cmd {
            id: AUTO_ORIENT,
            label: "Auto-Orient…",
            tooltip: "Find good build orientations (arrives with AM-04)",
            keywords: &["orientation", "optimise", "optimize", "orient"],
            group: GROUP_ANALYSE,
            pinned: false,
            action: CmdAction::Run(run_later),
            enabler: Enabler::When(later_orient),
        },
        Cmd {
            id: ARRANGE,
            label: "Arrange…",
            tooltip: "Pack the items on the plates (arrives with AM-05)",
            keywords: &["arrange", "pack", "nest", "layout"],
            group: GROUP_ANALYSE,
            pinned: false,
            action: CmdAction::Run(run_later),
            enabler: Enabler::When(later_arrange),
        },
        Cmd {
            id: SEND_TO_SLICER,
            label: "Send to Slicer",
            tooltip: "Write the active plate as 3MF and open it in the slicer (OrcaSlicer, Chitubox, …)",
            keywords: &[
                "slicer", "orca", "chitubox", "print", "handoff", "hand-off", "send",
            ],
            group: GROUP_OUTPUT,
            pinned: true,
            action: CmdAction::Run(run_send),
            enabler: Enabler::When(has_items),
        },
        Cmd {
            id: EXPORT_3MF,
            label: "Export Plate 3MF…",
            tooltip: "Write the active plate to a 3MF file",
            keywords: &["3mf", "export", "save", "plate"],
            group: GROUP_OUTPUT,
            pinned: true,
            action: CmdAction::Run(run_export),
            enabler: Enabler::When(has_items),
        },
        Cmd {
            id: SETTINGS,
            label: "Manufacture Settings…",
            tooltip: "Slicer path, hand-off folder and user machine folder",
            keywords: &[
                "preferences",
                "slicer path",
                "handoff folder",
                "machines",
                "settings",
            ],
            group: GROUP_OUTPUT,
            pinned: false,
            action: CmdAction::Run(run_settings),
            enabler: Enabler::Always,
        },
    ]
}

/// Registers the module's workspace, tab, groups, commands, browser provider, layer and panel.
pub fn register(r: &mut Registrar<'_>) {
    let mut ws = WorkspaceDef::new("am", "Manufacture", 30);
    ws.default_panels = vec![PLATE_PANEL];
    ws.on_enter = Some(open_panel);
    r.workspace(ws);
    r.toolbar_tab(TabDef {
        id: AM_TAB,
        workspace: AM,
        label: "Additive",
        order: 0,
        contextual: None,
    });
    for (order, (id, label)) in [
        (GROUP_SETUP, "Setup"),
        (GROUP_PLACE, "Place"),
        (GROUP_ANALYSE, "Analyse"),
        (GROUP_OUTPUT, "Output"),
    ]
    .into_iter()
    .enumerate()
    {
        r.toolbar_group(GroupDef {
            id,
            tab: AM_TAB,
            label,
            order: order as i32 * 10,
        });
    }
    for (order, c) in (0i32..).zip(commands()) {
        r.command(
            CommandDef::new(c.id.0, c.label, c.action)
                .with_tooltip(c.tooltip)
                .with_keywords(c.keywords)
                .with_enabler(c.enabler)
                .placed(Placement::Toolbar {
                    group: c.group,
                    pinned: c.pinned,
                    order: order * 10,
                }),
        );
    }
    r.browser_provider(Box::new(SetupBrowser));
    r.doc_layer(crate::layer::DocLayerDef {
        id: "am.plate",
        create: layer::create,
    });
    r.panel(PanelDef {
        id: PLATE_PANEL,
        title: "Plate",
        icon: crate::module::IconId::NONE,
        slot: DockSlot::Right,
        workspaces: Some(vec![AM]),
        create: browser::create_panel,
    });
}
