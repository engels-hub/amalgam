//! Assembly UI (lane E, S06 §7, ASM-10).
//!
//! - **Browser.** Every component row has an activate radio button while the design has
//!   occurrences, and the active component is emphasised. Grounded occurrences show a pin,
//!   occurrences with an uncaptured move a pending marker. The root shows "Positions out of
//!   date" while [`amg_doc::joints::out_of_date`] is non-empty; clicking it captures the
//!   positions.
//! - **Row menus.** Component rows: New Component, Activate, Paste; occurrences also Ground /
//!   Unground, Copy, Move to Component and Delete. Bodies and Bodies folders: Create
//!   Components from Bodies; bodies also Move to Component. A menu opened on a selected row
//!   acts on the whole selection of that kind.
//! - **Commands** ([`cmd`]) do the same for the selection, from S-key search and the marking
//!   menu's overflow list.
//! - **Active component.** [`Document::set_active_component`]; tools read it from
//!   [`CmdEnv::active_component`] as the context of new features.
//! - **Timeline.** Icons and names of the assembly feature kinds.
//!
//! [`Document::set_active_component`]: amg_doc::Document::set_active_component

mod actions;
mod browser;
mod state;

use amg_doc::DesignMode;

pub use actions::{selected_bodies, selected_occurrences};
pub use state::{AssemblyLayer, Clip, LAYER_ID, OutOfDateFn};

use crate::app::{AppCx, CmdEnv};
use crate::command::{CmdAction, CmdId, CommandDef, Enablement, Enabler};
use crate::layer::DocLayerDef;
use crate::marking::{ContextMenuProvider, ContextTarget};
use crate::module::{IconId, Registrar, UiModule};
use crate::timeline::{EditAction, FeatureUiInfo};

inventory::submit! { UiModule { id: "assembly", register } }

/// Command ids.
pub mod cmd {
    use crate::command::CmdId;
    /// New Component in the active component.
    pub const NEW_COMPONENT: CmdId = CmdId("assembly.new_component");
    /// Activate the selected occurrence's component.
    pub const ACTIVATE: CmdId = CmdId("assembly.activate");
    /// Ground the selected occurrences.
    pub const GROUND: CmdId = CmdId("assembly.ground");
    /// Unground the selected occurrences.
    pub const UNGROUND: CmdId = CmdId("assembly.unground");
    /// Create Components from the selected bodies.
    pub const FROM_BODIES: CmdId = CmdId("assembly.components_from_bodies");
    /// Copy the selected occurrences.
    pub const COPY: CmdId = CmdId("assembly.copy");
    /// Paste into the active component.
    pub const PASTE: CmdId = CmdId("assembly.paste");
    /// Delete the selected occurrences.
    pub const DELETE: CmdId = CmdId("assembly.delete");
    /// Capture Position.
    pub const CAPTURE: CmdId = CmdId("assembly.capture_position");

    /// Every command of the module.
    pub const ALL: [CmdId; 9] = [
        NEW_COMPONENT,
        ACTIVATE,
        GROUND,
        UNGROUND,
        FROM_BODIES,
        COPY,
        PASTE,
        DELETE,
        CAPTURE,
    ];
}

/// Menu entries and accessible names of the row items (followed by the row's label where
/// they belong to one component or occurrence).
pub mod labels {
    /// Menu: New Component.
    pub const NEW_COMPONENT: &str = "New Component";
    /// Menu: Activate; radio: "Activate <row>".
    pub const ACTIVATE: &str = "Activate";
    /// Menu: Ground.
    pub const GROUND: &str = "Ground";
    /// Menu: Unground.
    pub const UNGROUND: &str = "Unground";
    /// Menu: Create Components from Bodies.
    pub const FROM_BODIES: &str = "Create Components from Bodies";
    /// Menu: Copy.
    pub const COPY: &str = "Copy";
    /// Menu: Paste.
    pub const PASTE: &str = "Paste";
    /// Menu: Delete.
    pub const DELETE: &str = "Delete";
    /// Submenu: Move to Component.
    pub const MOVE_TO: &str = "Move to Component";
    /// Marker: "Grounded <row>".
    pub const GROUNDED: &str = "Grounded";
    /// Marker: "Position not captured <row>".
    pub const PENDING: &str = "Position not captured";
    /// The root badge; its action is Capture Position.
    pub const OUT_OF_DATE: &str = "Positions out of date: Capture Position";
}

/// The icons this module adds to the UI-23 registry (Tabler file stems).
pub mod icon_ids {
    use crate::module::IconId;
    /// Grounded marker and Ground.
    pub const GROUNDED: IconId = IconId("pin");
    /// Unground.
    pub const UNGROUND: IconId = IconId("pinned-off");
    /// The chosen radio button and Activate.
    pub const ACTIVE: IconId = crate::icons::ids::CIRCLE_CENTER;
    /// An unchosen radio button.
    pub const INACTIVE: IconId = IconId("circle");
    /// Pending-position marker.
    pub const PENDING: IconId = IconId("arrows-move");
    /// "Positions out of date" badge.
    pub const OUT_OF_DATE: IconId = IconId("alert-triangle");
    /// Capture Position.
    pub const CAPTURE: IconId = IconId("map-pin-check");
    /// Copy.
    pub const COPY: IconId = IconId("copy");
    /// Paste.
    pub const PASTE: IconId = IconId("clipboard");
    /// Create Components from Bodies.
    pub const FROM_BODIES: IconId = IconId("box-multiple");
}

macro_rules! svg {
    ($name:literal) => {
        include_bytes!(concat!("../../assets/icons/", $name, ".svg"))
    };
}

const ICONS: &[(IconId, &[u8])] = &[
    (icon_ids::GROUNDED, svg!("pin")),
    (icon_ids::UNGROUND, svg!("pinned-off")),
    (icon_ids::INACTIVE, svg!("circle")),
    (icon_ids::PENDING, svg!("arrows-move")),
    (icon_ids::OUT_OF_DATE, svg!("alert-triangle")),
    (icon_ids::CAPTURE, svg!("map-pin-check")),
    (icon_ids::COPY, svg!("copy")),
    (icon_ids::PASTE, svg!("clipboard")),
    (icon_ids::FROM_BODIES, svg!("box-multiple")),
];

fn need(ok: bool, reason: &'static str) -> Enablement {
    if ok {
        Enablement::Enabled
    } else {
        Enablement::disabled(reason)
    }
}

/// The component of the one selected occurrence.
fn selected_component(env: &CmdEnv<'_>) -> Option<amg_doc::ComponentId> {
    match selected_occurrences(env).as_slice() {
        [o] => env.design.occurrence(*o).map(|o| o.component),
        _ => None,
    }
}

fn any_selected_grounded(env: &CmdEnv<'_>, grounded: bool) -> bool {
    selected_occurrences(env).iter().any(|o| {
        env.design
            .occurrence(*o)
            .is_some_and(|d| d.grounded == grounded)
    })
}

fn can_activate(env: &CmdEnv<'_>) -> Enablement {
    need(
        selected_component(env).is_some_and(|c| c != env.active_component),
        "Select one occurrence of a component that is not active",
    )
}

fn can_ground(env: &CmdEnv<'_>) -> Enablement {
    need(
        any_selected_grounded(env, false),
        "Select an occurrence that is not grounded",
    )
}

fn can_unground(env: &CmdEnv<'_>) -> Enablement {
    need(
        any_selected_grounded(env, true),
        "Select a grounded occurrence",
    )
}

fn has_occurrence(env: &CmdEnv<'_>) -> Enablement {
    need(
        !selected_occurrences(env).is_empty(),
        "Select an occurrence",
    )
}

fn can_paste(env: &CmdEnv<'_>) -> Enablement {
    need(
        !actions::pastable(env, env.active_component).is_empty(),
        "Copy an occurrence that can be placed in the active component",
    )
}

fn can_create(env: &CmdEnv<'_>) -> Enablement {
    if let Some(reason) = actions::bodies_blocked(env) {
        return Enablement::disabled(reason);
    }
    need(!selected_bodies(env).is_empty(), "Select bodies")
}

fn can_capture(env: &CmdEnv<'_>) -> Enablement {
    let stale = env
        .doc_layer::<AssemblyLayer>()
        .is_some_and(|l| !l.out_of_date().is_empty());
    need(
        env.design.meta().mode == DesignMode::History
            && (env.design.has_pending_positions() || stale),
        "There is no position to capture",
    )
}

fn run_new_component(cx: &mut AppCx<'_>) {
    let parent = cx.env().active_component;
    actions::new_component(cx, parent);
}

fn run_activate(cx: &mut AppCx<'_>) {
    let component = selected_component(&cx.env());
    if let Some(c) = component {
        actions::activate(cx, c);
    }
}

fn run_ground(cx: &mut AppCx<'_>) {
    let occs = selected_occurrences(&cx.env());
    actions::set_grounded(cx, &occs, true);
}

fn run_unground(cx: &mut AppCx<'_>) {
    let occs = selected_occurrences(&cx.env());
    actions::set_grounded(cx, &occs, false);
}

fn run_from_bodies(cx: &mut AppCx<'_>) {
    let bodies = selected_bodies(&cx.env());
    actions::create_components(cx, &bodies);
}

fn run_copy(cx: &mut AppCx<'_>) {
    let occs = selected_occurrences(&cx.env());
    actions::copy(cx, &occs);
}

fn run_paste(cx: &mut AppCx<'_>) {
    let target = cx.env().active_component;
    actions::paste(cx, target);
}

fn run_delete(cx: &mut AppCx<'_>) {
    let occs = selected_occurrences(&cx.env());
    actions::delete(cx, occs);
}

/// Adds the commands for the selection to the marking menu's overflow list. The browser rows
/// have their own entries.
struct ViewportEntries;

impl ContextMenuProvider for ViewportEntries {
    fn fill(&self, env: &CmdEnv<'_>, target: &ContextTarget<'_>, out: &mut Vec<CmdId>) {
        if !matches!(target, ContextTarget::Viewport(_)) {
            return;
        }
        if !selected_occurrences(env).is_empty() {
            if can_activate(env).is_enabled() {
                out.push(cmd::ACTIVATE);
            }
            if can_ground(env).is_enabled() {
                out.push(cmd::GROUND);
            }
            if can_unground(env).is_enabled() {
                out.push(cmd::UNGROUND);
            }
            out.extend([cmd::COPY, cmd::DELETE]);
        }
        if can_paste(env).is_enabled() {
            out.push(cmd::PASTE);
        }
        if can_create(env).is_enabled() {
            out.push(cmd::FROM_BODIES);
        }
        if can_capture(env).is_enabled() {
            out.push(cmd::CAPTURE);
        }
    }
}

/// One command: id, label, icon, enablement, action.
type Row = (CmdId, &'static str, IconId, Enabler, fn(&mut AppCx<'_>));

/// Registers the module's icons, commands, browser rows, document layer and timeline info.
pub fn register(r: &mut Registrar<'_>) {
    amg_features::assembly::link();
    for (id, bytes) in ICONS {
        r.icon(*id, bytes);
    }
    r.doc_layer(DocLayerDef {
        id: LAYER_ID,
        create: || Box::new(AssemblyLayer::default()),
    });
    r.browser_provider(Box::new(browser::AssemblyRows));
    r.context_menu(Box::new(ViewportEntries));
    let commands: [Row; 9] = [
        (
            cmd::NEW_COMPONENT,
            labels::NEW_COMPONENT,
            crate::icons::ids::COMPONENT,
            Enabler::Always,
            run_new_component,
        ),
        (
            cmd::ACTIVATE,
            "Activate Component",
            icon_ids::ACTIVE,
            Enabler::When(can_activate),
            run_activate,
        ),
        (
            cmd::GROUND,
            labels::GROUND,
            icon_ids::GROUNDED,
            Enabler::When(can_ground),
            run_ground,
        ),
        (
            cmd::UNGROUND,
            labels::UNGROUND,
            icon_ids::UNGROUND,
            Enabler::When(can_unground),
            run_unground,
        ),
        (
            cmd::FROM_BODIES,
            labels::FROM_BODIES,
            icon_ids::FROM_BODIES,
            Enabler::When(can_create),
            run_from_bodies,
        ),
        (
            cmd::COPY,
            "Copy Occurrence",
            icon_ids::COPY,
            Enabler::When(has_occurrence),
            run_copy,
        ),
        (
            cmd::PASTE,
            "Paste Occurrence",
            icon_ids::PASTE,
            Enabler::When(can_paste),
            run_paste,
        ),
        (
            cmd::DELETE,
            "Delete Occurrence",
            crate::icons::ids::DELETE,
            Enabler::When(has_occurrence),
            run_delete,
        ),
        (
            cmd::CAPTURE,
            "Capture Position",
            icon_ids::CAPTURE,
            Enabler::When(can_capture),
            actions::capture,
        ),
    ];
    for (id, label, icon, enabled, run) in commands {
        r.command(
            CommandDef::new(id.0, label, CmdAction::Run(run))
                .with_icon(icon)
                .with_enabler(enabled)
                .with_keywords(&["assembly", "component"]),
        );
    }
    for (kind, display_name, icon) in [
        (
            amg_doc::positions::CAPTURE_POSITION_KIND,
            "Capture Position",
            icon_ids::CAPTURE,
        ),
        (
            amg_features::assembly::BODIES_TO_COMPONENTS_KIND,
            "Move Bodies",
            icon_ids::FROM_BODIES,
        ),
    ] {
        r.feature_ui(FeatureUiInfo {
            kind,
            display_name,
            icon,
            edit: EditAction::NotEditable,
        });
    }
}
