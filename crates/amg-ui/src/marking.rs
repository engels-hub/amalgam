//! The right-click marking menu and the context-menu provider API (spec §5, note §4, UI-21).
//!
//! **Layout.** Eight wedges ([`Wedge::N`] … [`Wedge::NW`]) around the point where the right
//! button went down, "Repeat <last command>" at the top centre (above N) and an overflow list
//! below S, as in Fusion.
//!
//! **Contents.** [`MarkingMenuProvider`]s fill a [`MarkingMenuBuilder`], highest
//! [`MarkingMenuProvider::priority`] first (ties in registration order). The first claim of a
//! wedge wins; a later claim of a taken wedge goes to the overflow list. Claims of commands
//! that are not registered are skipped, so a provider can name commands other lanes add later
//! ([`MarkingMenuBuilder::wedge_any`] takes the first registered of several). After the
//! providers, the [`ContextMenuProvider`]s add their entries for
//! [`ContextTarget::Viewport`] (the hovered item) to the overflow list. The framework's own
//! provider ([`FrameworkMarkingMenu`], priority 0) fills the defaults: undo, redo,
//! press-pull / extrude, move, sketch, delete and measure.
//!
//! **Gestures** (note §4), with the right button and no modifier, when the active tool does
//! not take the press:
//! - *Click:* press and release within [`GESTURE_PT`] and before [`HOLD`] opens the menu at
//!   the press point; it stays open. Clicking a wedge (its button, or anywhere in its sector
//!   of the viewport) runs its command; a press elsewhere or `Esc` closes it.
//! - *Gesture:* a press followed by a drag of more than [`GESTURE_PT`] runs, on release, the
//!   command of the wedge in the drag direction. The menu never opens (a trail and the wedge
//!   label follow the pointer meanwhile).
//! - *Hold:* holding for [`HOLD`] with less than [`CLICK_SLOP_PT`] of movement opens the menu
//!   while the button is down; releasing over a wedge runs it, releasing in the centre keeps
//!   the menu open as in click mode.
//!
//! Commands run like toolbar commands ([`AmalgamApp::request_command`]: end of frame,
//! enablement checked, recorded for search recency and "Repeat").
//!
//! **Context menus.** The browser row menu ([`ContextTarget::Browser`]) and the timeline node
//! menu ([`ContextTarget::Timeline`]) append the commands of every [`ContextMenuProvider`]
//! below their own entries ([`context_menu_ui`]).

use std::time::{Duration, Instant};

use egui::{Align2, Pos2, Vec2};

use crate::app::{AmalgamApp, CmdEnv};
use crate::browser::NodeKey;
use crate::command::{CmdId, Enablement};
use crate::input::ViewportEvent;
use crate::registry::UiRegistry;
use crate::search::Recency;
use crate::select::SelItem;
use crate::tool::ToolState;

/// A press and release closer than this (points) is a click, farther is a gesture.
pub const GESTURE_PT: f32 = 12.0;
/// A held press that moved less than this (points) opens the menu after [`HOLD`].
pub const CLICK_SLOP_PT: f32 = 4.0;
/// Holding the right button this long without moving opens the menu and waits.
pub const HOLD: Duration = Duration::from_millis(300);
/// Distance of the wedge buttons from the centre, points.
pub const RADIUS_PT: f32 = 90.0;
/// Within this distance of the centre no wedge is hot, points.
pub const DEAD_ZONE_PT: f32 = 20.0;
/// Beyond this distance of the centre no wedge is hot, points.
pub const OUTER_PT: f32 = 260.0;
/// Commands "Repeat" never offers (they are not what the user would repeat).
const NOT_REPEATED: [CmdId; 3] = [CmdId("edit.undo"), CmdId("edit.redo"), CmdId("app.search")];

/// One of the eight directions of the radial menu.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Wedge {
    /// Up.
    N,
    /// Up right.
    NE,
    /// Right.
    E,
    /// Down right.
    SE,
    /// Down.
    S,
    /// Down left.
    SW,
    /// Left.
    W,
    /// Up left.
    NW,
}

impl Wedge {
    /// All wedges, clockwise from N.
    pub const ALL: [Wedge; 8] = [
        Wedge::N,
        Wedge::NE,
        Wedge::E,
        Wedge::SE,
        Wedge::S,
        Wedge::SW,
        Wedge::W,
        Wedge::NW,
    ];

    /// Index in [`Wedge::ALL`].
    pub fn index(self) -> usize {
        self as usize
    }

    /// The unit direction in screen space (y down).
    pub fn dir(self) -> Vec2 {
        // Clockwise from N in steps of 45°; screen y points down.
        #[allow(clippy::cast_precision_loss, reason = "index < 8")]
        let a = self.index() as f32 * std::f32::consts::FRAC_PI_4;
        egui::vec2(a.sin(), -a.cos())
    }

    /// The wedge whose 45° sector contains direction `d` (screen space, y down); `None` for a
    /// zero vector.
    pub fn from_dir(d: Vec2) -> Option<Wedge> {
        if d.length_sq() <= f32::EPSILON {
            return None;
        }
        // Clockwise angle from N.
        let a = d.x.atan2(-d.y).rem_euclid(std::f32::consts::TAU);
        #[allow(
            clippy::cast_possible_truncation,
            clippy::cast_sign_loss,
            reason = "a ∈ [0, 2π): the rounded sector is 0..=8"
        )]
        let i = (a / std::f32::consts::FRAC_PI_4).round() as usize % 8;
        Some(Wedge::ALL[i])
    }

    /// "N", "NE", …
    pub fn name(self) -> &'static str {
        match self {
            Wedge::N => "N",
            Wedge::NE => "NE",
            Wedge::E => "E",
            Wedge::SE => "SE",
            Wedge::S => "S",
            Wedge::SW => "SW",
            Wedge::W => "W",
            Wedge::NW => "NW",
        }
    }

    /// Where the wedge's button sits relative to the centre, and which of its points is there.
    fn anchor(self, center: Pos2) -> (Pos2, Align2) {
        let at = center + self.dir() * RADIUS_PT;
        let align = match self {
            Wedge::N => Align2::CENTER_BOTTOM,
            Wedge::S => Align2::CENTER_TOP,
            Wedge::NE | Wedge::E | Wedge::SE => Align2::LEFT_CENTER,
            Wedge::NW | Wedge::W | Wedge::SW => Align2::RIGHT_CENTER,
        };
        (at, align)
    }
}

#[derive(Clone, Debug)]
enum Slot {
    Wedge(Wedge),
    Overflow,
}

/// What providers fill: wedge claims and overflow entries, resolved against the registry by
/// [`MarkingMenu::build`].
#[derive(Clone, Debug, Default)]
pub struct MarkingMenuBuilder {
    claims: Vec<(Slot, Vec<CmdId>)>,
}

impl MarkingMenuBuilder {
    /// An empty builder.
    pub fn new() -> MarkingMenuBuilder {
        MarkingMenuBuilder::default()
    }

    /// Claims wedge `w` for `cmd`. First provider (by priority) to claim a wedge wins; later
    /// ones go to overflow. An unregistered command is skipped.
    pub fn wedge(&mut self, w: Wedge, cmd: CmdId) {
        self.claims.push((Slot::Wedge(w), vec![cmd]));
    }

    /// Claims wedge `w` for the first of `cmds` that is registered (e.g. press-pull, else
    /// extrude). Nothing happens when none is registered.
    pub fn wedge_any(&mut self, w: Wedge, cmds: &[CmdId]) {
        self.claims.push((Slot::Wedge(w), cmds.to_vec()));
    }

    /// Adds `cmd` to the overflow list (below the wedges). An unregistered command is skipped.
    pub fn overflow(&mut self, cmd: CmdId) {
        self.claims.push((Slot::Overflow, vec![cmd]));
    }

    /// Number of claims so far.
    pub fn len(&self) -> usize {
        self.claims.len()
    }

    /// True before any claim.
    pub fn is_empty(&self) -> bool {
        self.claims.is_empty()
    }
}

/// Fills the marking menu for the current workspace, context, selection and hover.
pub trait MarkingMenuProvider: 'static {
    /// Higher fills first (the framework default provider is 0).
    fn priority(&self) -> i32;
    /// Claims wedges and overflow entries.
    fn fill(&self, env: &CmdEnv<'_>, out: &mut MarkingMenuBuilder);
}

/// What a context menu is opened on.
#[derive(Clone, Copy, Debug)]
pub enum ContextTarget<'a> {
    /// The viewport, with the hovered item (the marking menu's overflow list).
    Viewport(Option<&'a SelItem>),
    /// A browser row.
    Browser(&'a NodeKey),
    /// A timeline node.
    Timeline(amg_doc::FeatureId),
}

/// Adds commands to the browser, timeline and viewport context menus.
pub trait ContextMenuProvider: 'static {
    /// Appends the commands for `target` (unregistered ids are skipped when drawn).
    fn fill(&self, env: &CmdEnv<'_>, target: &ContextTarget<'_>, out: &mut Vec<CmdId>);
}

/// A resolved marking menu: what each wedge, the overflow list and "Repeat" run.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct MarkingMenu {
    /// By [`Wedge::index`].
    pub wedges: [Option<CmdId>; 8],
    /// Below the wedges, in order.
    pub overflow: Vec<CmdId>,
    /// The most recent repeatable command ("Repeat <label>").
    pub repeat: Option<CmdId>,
}

impl MarkingMenu {
    /// Runs the registry's marking-menu providers (highest priority first), then its context
    /// providers for the viewport (`env.hovered`), and picks "Repeat" from `recency`.
    pub fn build(registry: &UiRegistry, env: &CmdEnv<'_>, recency: &Recency) -> MarkingMenu {
        let mut b = MarkingMenuBuilder::new();
        for p in registry.marking_menu_providers() {
            p.fill(env, &mut b);
        }
        for id in context_menu_commands(registry, env, &ContextTarget::Viewport(env.hovered)) {
            b.overflow(id);
        }
        let mut menu = MarkingMenu::resolve(b, registry);
        menu.repeat = recency
            .iter()
            .find(|id| !NOT_REPEATED.contains(id) && registry.command(*id).is_some());
        menu
    }

    /// Resolves the claims of `b` against `registry` (see [`MarkingMenuBuilder::wedge`]).
    pub fn resolve(b: MarkingMenuBuilder, registry: &UiRegistry) -> MarkingMenu {
        let mut menu = MarkingMenu::default();
        for (slot, cmds) in b.claims {
            let Some(cmd) = cmds
                .into_iter()
                .find(|c| registry.command(*c).is_some() && !menu.contains(*c))
            else {
                continue;
            };
            match slot {
                Slot::Wedge(w) if menu.wedges[w.index()].is_none() => {
                    menu.wedges[w.index()] = Some(cmd);
                }
                Slot::Wedge(_) | Slot::Overflow => menu.overflow.push(cmd),
            }
        }
        menu
    }

    /// The command of wedge `w`.
    pub fn wedge(&self, w: Wedge) -> Option<CmdId> {
        self.wedges[w.index()]
    }

    /// True if `cmd` is in a wedge or the overflow list.
    pub fn contains(&self, cmd: CmdId) -> bool {
        self.wedges.contains(&Some(cmd)) || self.overflow.contains(&cmd)
    }

    /// True without any entry (no wedge, no overflow, nothing to repeat).
    pub fn is_empty(&self) -> bool {
        self.wedges.iter().all(Option::is_none) && self.overflow.is_empty() && self.repeat.is_none()
    }
}

/// The commands every context provider adds for `target`, in registration order, without
/// duplicates and without unregistered ids.
pub fn context_menu_commands(
    registry: &UiRegistry,
    env: &CmdEnv<'_>,
    target: &ContextTarget<'_>,
) -> Vec<CmdId> {
    let mut raw = Vec::new();
    for p in registry.context_menu_providers() {
        p.fill(env, target, &mut raw);
    }
    let mut out: Vec<CmdId> = Vec::with_capacity(raw.len());
    for id in raw {
        if registry.command(id).is_some() && !out.contains(&id) {
            out.push(id);
        }
    }
    out
}

/// Draws the context providers' commands for `target` into an open context menu (after a
/// separator; nothing when there are none). Disabled commands show their reason on hover.
/// Returns the clicked command (the menu is closed); the caller runs it.
pub fn context_menu_ui(
    ui: &mut egui::Ui,
    registry: &UiRegistry,
    env: &CmdEnv<'_>,
    target: &ContextTarget<'_>,
) -> Option<CmdId> {
    let cmds = context_menu_commands(registry, env, target);
    if cmds.is_empty() {
        return None;
    }
    ui.separator();
    let mut clicked = None;
    for id in cmds {
        if command_button(ui, registry, env, id, false, None).clicked() {
            clicked = Some(id);
            ui.close();
        }
    }
    clicked
}

/// A command as a button: its label (or `text`), disabled with the reason as tooltip.
fn command_button(
    ui: &mut egui::Ui,
    registry: &UiRegistry,
    env: &CmdEnv<'_>,
    id: CmdId,
    hot: bool,
    text: Option<String>,
) -> egui::Response {
    let Some(def) = registry.command(id) else {
        return ui.label("");
    };
    let label = text.unwrap_or_else(|| def.label.to_string());
    let en = def.enablement(env);
    let mut r = ui.add_enabled(
        en == Enablement::Enabled,
        crate::icons::button(ui, def.icon, label, en == Enablement::Enabled).selected(hot),
    );
    if let Enablement::Disabled(reason) = &en {
        r = r.on_disabled_hover_text(reason.as_ref());
    } else if !def.tooltip.is_empty() {
        r = r.on_hover_text(def.tooltip.as_ref());
    }
    r
}

/// The framework's default entries (priority 0): undo W, redo E, sketch N, press-pull /
/// extrude NE, move SW, delete S, measure SE. Ids of other lanes' commands are skipped until
/// they are registered.
#[derive(Clone, Copy, Debug, Default)]
pub struct FrameworkMarkingMenu;

/// Command ids the framework default provider places (the first registered of each list).
pub mod defaults {
    use crate::command::CmdId;
    /// Undo (W).
    pub const UNDO: CmdId = CmdId("edit.undo");
    /// Redo (E).
    pub const REDO: CmdId = CmdId("edit.redo");
    /// Create Sketch (N).
    pub const SKETCH: &[CmdId] = &[CmdId("sketch.sketch"), CmdId("feature.sketch.sketch")];
    /// Press Pull, else Extrude (NE).
    pub const PRESS_PULL: &[CmdId] = &[
        CmdId("solid.press_pull"),
        CmdId("feature.solid.press_pull"),
        CmdId("feature.solid.extrude"),
    ];
    /// Move / Copy (SW).
    pub const MOVE: &[CmdId] = &[CmdId("solid.move"), CmdId("feature.solid.move")];
    /// Delete (S).
    pub const DELETE: &[CmdId] = &[CmdId("edit.delete")];
    /// Measure (SE, S05 INS-08).
    pub const MEASURE: &[CmdId] = &[CmdId("inspect.measure")];
}

impl MarkingMenuProvider for FrameworkMarkingMenu {
    fn priority(&self) -> i32 {
        0
    }

    fn fill(&self, _env: &CmdEnv<'_>, out: &mut MarkingMenuBuilder) {
        out.wedge(Wedge::W, defaults::UNDO);
        out.wedge(Wedge::E, defaults::REDO);
        out.wedge_any(Wedge::N, defaults::SKETCH);
        out.wedge_any(Wedge::NE, defaults::PRESS_PULL);
        out.wedge_any(Wedge::SW, defaults::MOVE);
        out.wedge_any(Wedge::S, defaults::DELETE);
        out.wedge_any(Wedge::SE, defaults::MEASURE);
    }
}

// ---- state machine ---------------------------------------------------------------------------

/// The open marking menu.
#[derive(Clone, Debug, PartialEq)]
pub struct OpenMarkingMenu {
    /// Centre (where the right button went down), points.
    pub center: Pos2,
    /// Its entries.
    pub menu: MarkingMenu,
    /// Opened by holding: the right button is still down, and its release over a wedge runs it.
    pub held: bool,
}

impl OpenMarkingMenu {
    /// The wedge whose sector contains `pos`, between [`DEAD_ZONE_PT`] and [`OUTER_PT`].
    pub fn hot_wedge(&self, pos: Pos2) -> Option<Wedge> {
        let d = pos - self.center;
        let len = d.length();
        if !(DEAD_ZONE_PT..=OUTER_PT).contains(&len) {
            return None;
        }
        Wedge::from_dir(d)
    }

    /// The command of the hot wedge at `pos`.
    fn hot_command(&self, pos: Pos2) -> Option<CmdId> {
        self.hot_wedge(pos).and_then(|w| self.menu.wedge(w))
    }
}

/// A right press being tracked (before it became a click, gesture or hold).
#[derive(Clone, Debug)]
struct Gesture {
    origin: Pos2,
    pos: Pos2,
    at: Instant,
    gesturing: bool,
    menu: MarkingMenu,
}

/// Per-document marking-menu state (in the router state, [`ToolState`]).
#[derive(Debug, Default)]
pub(crate) struct MarkingState {
    gesture: Option<Gesture>,
    open: Option<OpenMarkingMenu>,
    /// Buttons whose remaining events (up to their click / drag end) are dropped: the press
    /// that closed the menu, the release that ran a gesture.
    swallow: Vec<egui::PointerButton>,
}

impl MarkingState {
    /// Closes the menu or drops a tracked gesture; true if there was one.
    pub(crate) fn close(&mut self) -> bool {
        let had = self.open.is_some() || self.gesture.is_some();
        self.open = None;
        self.gesture = None;
        had
    }

    /// The open menu.
    pub(crate) fn open(&self) -> Option<&OpenMarkingMenu> {
        self.open.as_ref()
    }
}

fn button_of(ev: &ViewportEvent) -> Option<egui::PointerButton> {
    match ev {
        ViewportEvent::Press { button, .. }
        | ViewportEvent::Release { button, .. }
        | ViewportEvent::Click { button, .. }
        | ViewportEvent::DragStart { button, .. }
        | ViewportEvent::Drag { button, .. }
        | ViewportEvent::DragEnd { button, .. } => Some(*button),
        _ => None,
    }
}

/// True for a press that starts a marking gesture: right button, no modifier.
pub(crate) fn starts_gesture(ev: &ViewportEvent) -> bool {
    matches!(ev, ViewportEvent::Press { button: egui::PointerButton::Secondary, mods, .. }
        if !mods.shift && !mods.ctrl && !mods.alt && !mods.command)
}

/// Starts tracking a right press at `pos` (the active tool ignored it). The menu is built now,
/// so it reflects the selection and hover at the press.
pub(crate) fn begin(app: &mut AmalgamApp, ts: &mut ToolState, pos: Pos2) {
    let menu = {
        let doc = &app.docs[app.active];
        let env = app.env_for(
            doc,
            ts.frame.hover.as_ref().map(|h| &h.item),
            ts.active.as_ref().map(|a| a.id),
        );
        MarkingMenu::build(&app.registry, &env, &app.recency)
    };
    ts.marking.open = None;
    ts.marking.gesture = Some(Gesture {
        origin: pos,
        pos,
        at: Instant::now(),
        gesturing: false,
        menu,
    });
}

/// The popup step of the routing chain (note §4: popups come before the tool). Returns true
/// if the marking menu used the event.
pub(crate) fn intercept(app: &mut AmalgamApp, ts: &mut ToolState, ev: &ViewportEvent) -> bool {
    const RIGHT: egui::PointerButton = egui::PointerButton::Secondary;
    let m = &mut ts.marking;
    if let Some(b) = button_of(ev)
        && m.swallow.contains(&b)
    {
        if matches!(
            ev,
            ViewportEvent::Click { .. } | ViewportEvent::DragEnd { .. }
        ) {
            m.swallow.retain(|s| *s != b);
        }
        return true;
    }
    if let Some(g) = &mut m.gesture {
        return match ev {
            ViewportEvent::Hover { pos, .. } => {
                g.pos = *pos;
                g.gesturing |= g.origin.distance(*pos) > GESTURE_PT;
                false
            }
            ViewportEvent::Drag { button, pos, .. } if *button == RIGHT => {
                g.pos = *pos;
                g.gesturing |= g.origin.distance(*pos) > GESTURE_PT;
                true
            }
            ViewportEvent::DragStart { button, .. } if *button == RIGHT => true,
            ViewportEvent::Release { button, pos, .. } if *button == RIGHT => {
                let Some(g) = m.gesture.take() else {
                    return true;
                };
                let d = *pos - g.origin;
                if g.gesturing || d.length() > GESTURE_PT {
                    if let Some(cmd) = Wedge::from_dir(d).and_then(|w| g.menu.wedge(w)) {
                        tracing::debug!(%cmd, "marking menu gesture");
                        app.request_command(cmd);
                    }
                } else if !g.menu.is_empty() {
                    m.open = Some(OpenMarkingMenu {
                        center: g.origin,
                        menu: g.menu,
                        held: false,
                    });
                }
                m.swallow.push(RIGHT);
                true
            }
            _ => false,
        };
    }
    if let Some(o) = &mut m.open {
        return match ev {
            ViewportEvent::Press { button, pos, .. } => {
                let cmd = o.hot_command(*pos);
                m.open = None;
                if let Some(cmd) = cmd {
                    app.request_command(cmd);
                }
                m.swallow.push(*button);
                true
            }
            ViewportEvent::Release { button, pos, .. } if *button == RIGHT && o.held => {
                o.held = false;
                if let Some(cmd) = o.hot_command(*pos) {
                    m.open = None;
                    app.request_command(cmd);
                }
                m.swallow.push(RIGHT);
                true
            }
            ViewportEvent::DragStart { button, .. } | ViewportEvent::Drag { button, .. }
                if *button == RIGHT && o.held =>
            {
                true
            }
            _ => false,
        };
    }
    false
}

// ---- app integration -----------------------------------------------------------------------

impl AmalgamApp {
    /// The open marking menu of the active document (UI-21).
    pub fn marking_menu(&self) -> Option<&OpenMarkingMenu> {
        self.docs[self.active].tools.marking.open()
    }

    /// Opens the marking menu at `center` as a right click there would (hosts and tests).
    pub fn open_marking_menu(&mut self, center: Pos2) {
        let doc = &self.docs[self.active];
        let ts = &doc.tools;
        let env = self.env_for(
            doc,
            ts.frame.hover.as_ref().map(|h| &h.item),
            ts.active.as_ref().map(|a| a.id),
        );
        let menu = MarkingMenu::build(&self.registry, &env, &self.recency);
        let m = &mut self.docs[self.active].tools.marking;
        m.gesture = None;
        m.open = Some(OpenMarkingMenu {
            center,
            menu,
            held: false,
        });
        if let Some(ctx) = &self.ctx {
            ctx.request_repaint();
        }
    }

    /// Closes the marking menu of the active document; true if one was open.
    pub fn close_marking_menu(&mut self) -> bool {
        self.docs[self.active].tools.marking.close()
    }

    /// Draws the marking menu (or the gesture trail) and runs what was clicked. Also turns a
    /// held right press into the open menu after [`HOLD`].
    pub(crate) fn draw_marking_menu(&mut self, ctx: &egui::Context) {
        let active = self.active;
        let m = &mut self.docs[active].tools.marking;
        if let Some(g) = &m.gesture
            && !g.gesturing
        {
            let held = g.at.elapsed();
            if g.origin.distance(g.pos) < CLICK_SLOP_PT && held >= HOLD {
                let Some(g) = m.gesture.take() else {
                    return;
                };
                m.open = Some(OpenMarkingMenu {
                    center: g.origin,
                    menu: g.menu,
                    held: true,
                });
                ctx.request_repaint();
            } else {
                ctx.request_repaint_after(HOLD.saturating_sub(held));
            }
        }
        if let Some(g) = &m.gesture
            && g.gesturing
        {
            let label = Wedge::from_dir(g.pos - g.origin)
                .and_then(|w| g.menu.wedge(w))
                .and_then(|c| self.registry.command(c))
                .map(|d| d.label.to_string());
            paint_trail(ctx, g.origin, g.pos, label, &self.tokens);
            return;
        }
        let Some(open) = self.docs[active].tools.marking.open.clone() else {
            return;
        };
        let pointer = ctx.input(|i| i.pointer.latest_pos());
        let hot = pointer.and_then(|p| open.hot_wedge(p));
        let doc = &self.docs[active];
        let env = crate::app::env_of_doc(self.workspace, doc, &self.jobs);
        let reg = &self.registry;
        let mut clicked: Option<CmdId> = None;
        let mut over_menu = false;
        let center = open.center;
        paint_hub(ctx, center, hot, &self.tokens);
        let area = |id: &str, at: Pos2, align: Align2| {
            egui::Area::new(egui::Id::new(("amg_marking_menu", id.to_owned())))
                .order(egui::Order::Foreground)
                .fixed_pos(at)
                .pivot(align)
                .interactable(true)
        };
        for w in Wedge::ALL {
            let Some(cmd) = open.menu.wedge(w) else {
                continue;
            };
            let (at, align) = w.anchor(center);
            let r = area(w.name(), at, align).show(ctx, |ui| {
                egui::Frame::popup(ui.style())
                    .inner_margin(egui::Margin::same(2))
                    .show(ui, |ui| {
                        command_button(ui, reg, &env, cmd, hot == Some(w), None).clicked()
                    })
                    .inner
            });
            over_menu |= r.response.contains_pointer();
            if r.inner {
                clicked = Some(cmd);
            }
        }
        if let Some(cmd) = open.menu.repeat
            && let Some(def) = reg.command(cmd)
        {
            let at = center + egui::vec2(0.0, -RADIUS_PT - 34.0);
            let text = format!("Repeat {}", def.label);
            let r = area("repeat", at, Align2::CENTER_BOTTOM).show(ctx, |ui| {
                egui::Frame::popup(ui.style())
                    .inner_margin(egui::Margin::same(2))
                    .show(ui, |ui| {
                        command_button(ui, reg, &env, cmd, false, Some(text)).clicked()
                    })
                    .inner
            });
            over_menu |= r.response.contains_pointer();
            if r.inner {
                clicked = Some(cmd);
            }
        }
        if !open.menu.overflow.is_empty() {
            let at = center + egui::vec2(0.0, RADIUS_PT + 34.0);
            let r = area("overflow", at, Align2::CENTER_TOP).show(ctx, |ui| {
                egui::Frame::menu(ui.style())
                    .show(ui, |ui| {
                        ui.set_min_width(140.0);
                        let mut c = None;
                        for &cmd in &open.menu.overflow {
                            let b = ui
                                .add_sized([ui.available_width(), 0.0], |ui: &mut egui::Ui| {
                                    command_button(ui, reg, &env, cmd, false, None)
                                });
                            if b.clicked() {
                                c = Some(cmd);
                            }
                        }
                        c
                    })
                    .inner
            });
            over_menu |= r.response.contains_pointer();
            if let Some(c) = r.inner {
                clicked = Some(c);
            }
        }
        let pressed_elsewhere = !over_menu && ctx.input(|i| i.pointer.any_pressed());
        let pointer_in_viewport = pointer.is_some_and(|p| {
            self.docs[active]
                .view
                .as_ref()
                .is_some_and(|v| v.rect.contains(p))
        });
        if let Some(cmd) = clicked {
            self.docs[active].tools.marking.open = None;
            self.request_command(cmd);
        } else if pressed_elsewhere && !pointer_in_viewport {
            // Presses in the viewport were handled by the router (`intercept`).
            self.docs[active].tools.marking.open = None;
            ctx.request_repaint();
        }
    }
}

/// The centre ring and the hot wedge's sector line.
fn paint_hub(
    ctx: &egui::Context,
    center: Pos2,
    hot: Option<Wedge>,
    tokens: &crate::theme::ThemeTokens,
) {
    let painter = ctx.layer_painter(egui::LayerId::new(
        egui::Order::Foreground,
        egui::Id::new("amg_marking_hub"),
    ));
    let stroke = egui::Stroke::new(1.5, tokens.accent);
    painter.circle_filled(center, DEAD_ZONE_PT, tokens.accent.gamma_multiply(0.10));
    painter.circle_stroke(center, DEAD_ZONE_PT, stroke);
    if let Some(w) = hot {
        let d = w.dir();
        painter.line_segment(
            [center + d * DEAD_ZONE_PT, center + d * (RADIUS_PT - 8.0)],
            egui::Stroke::new(2.0, tokens.hover),
        );
    }
}

/// Gesture mode: a line from the press point to the pointer and the target command's label.
fn paint_trail(
    ctx: &egui::Context,
    from: Pos2,
    to: Pos2,
    label: Option<String>,
    tokens: &crate::theme::ThemeTokens,
) {
    let painter = ctx.layer_painter(egui::LayerId::new(
        egui::Order::Foreground,
        egui::Id::new("amg_marking_trail"),
    ));
    painter.line_segment([from, to], egui::Stroke::new(2.0, tokens.accent));
    painter.circle_stroke(from, 4.0, egui::Stroke::new(1.5, tokens.accent));
    if let Some(text) = label {
        painter.text(
            to + egui::vec2(12.0, -12.0),
            Align2::LEFT_BOTTOM,
            text,
            egui::FontId::proportional(13.0),
            tokens.accent,
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::command::{CmdAction, CommandDef};
    use crate::module::{Registrar, UiModule};

    #[test]
    fn directions_map_to_their_wedge_and_back() {
        for w in Wedge::ALL {
            assert_eq!(Wedge::from_dir(w.dir()), Some(w), "{w:?}");
            assert_eq!(Wedge::from_dir(w.dir() * 50.0), Some(w));
        }
        assert_eq!(Wedge::from_dir(egui::vec2(0.0, -1.0)), Some(Wedge::N));
        assert_eq!(Wedge::from_dir(egui::vec2(1.0, 0.0)), Some(Wedge::E));
        assert_eq!(Wedge::from_dir(egui::vec2(0.0, 1.0)), Some(Wedge::S));
        assert_eq!(Wedge::from_dir(egui::vec2(-1.0, 0.0)), Some(Wedge::W));
        // 20° off E is still E; 25° off is NE.
        let a = 20f32.to_radians();
        assert_eq!(
            Wedge::from_dir(egui::vec2(a.cos(), -a.sin())),
            Some(Wedge::E)
        );
        let a = 25f32.to_radians();
        assert_eq!(
            Wedge::from_dir(egui::vec2(a.cos(), -a.sin())),
            Some(Wedge::NE)
        );
        assert_eq!(Wedge::from_dir(Vec2::ZERO), None);
    }

    fn cmds(r: &mut Registrar<'_>) {
        for id in ["t.a", "t.b", "t.c", "t.d"] {
            r.command(CommandDef::new(id, id, CmdAction::Run(|_| {})));
        }
    }

    #[test]
    fn first_claim_wins_later_ones_overflow_and_unknown_ids_are_skipped() {
        let m = UiModule {
            id: "t",
            register: cmds,
        };
        let reg = UiRegistry::from_modules([&m]);
        let mut b = MarkingMenuBuilder::new();
        b.wedge(Wedge::N, CmdId("t.a"));
        b.wedge(Wedge::N, CmdId("t.b")); // taken → overflow
        b.wedge(Wedge::E, CmdId("nope.x")); // unregistered → skipped, E stays free
        b.wedge_any(Wedge::E, &[CmdId("nope.y"), CmdId("t.c")]);
        b.overflow(CmdId("t.a")); // already placed → skipped
        b.overflow(CmdId("t.d"));
        let menu = MarkingMenu::resolve(b, &reg);
        assert_eq!(menu.wedge(Wedge::N), Some(CmdId("t.a")));
        assert_eq!(menu.wedge(Wedge::E), Some(CmdId("t.c")));
        assert_eq!(menu.overflow, [CmdId("t.b"), CmdId("t.d")]);
        assert!(
            Wedge::ALL
                .iter()
                .filter(|w| !matches!(w, Wedge::N | Wedge::E))
                .all(|w| menu.wedge(*w).is_none())
        );
    }

    #[test]
    fn hot_wedge_respects_the_dead_zone_and_outer_radius() {
        let o = OpenMarkingMenu {
            center: egui::pos2(100.0, 100.0),
            menu: MarkingMenu::default(),
            held: false,
        };
        assert_eq!(o.hot_wedge(egui::pos2(105.0, 100.0)), None);
        assert_eq!(o.hot_wedge(egui::pos2(100.0, 40.0)), Some(Wedge::N));
        assert_eq!(o.hot_wedge(egui::pos2(100.0 + OUTER_PT + 1.0, 100.0)), None);
    }
}
