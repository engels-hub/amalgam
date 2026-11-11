//! Dockable panels contributed by modules (spec S11 Interfaces §A `amg_ui::panel`).

use crate::app::{AppCx, CmdEnv};
use crate::module::{IconId, PanelId, WorkspaceId};

/// Where a panel opens in the dock area.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum DockSlot {
    /// A split left of the viewport.
    Left,
    /// A split right of the viewport.
    Right,
    /// A split below the viewport.
    Bottom,
    /// A tab next to the viewport tab.
    CenterTab,
    /// A floating dock window.
    Floating,
}

/// A panel registration.
#[derive(Clone, Debug)]
pub struct PanelDef {
    /// Unique id.
    pub id: PanelId,
    /// Tab title.
    pub title: &'static str,
    /// Icon.
    pub icon: IconId,
    /// Initial slot.
    pub slot: DockSlot,
    /// Workspaces the panel may show in; `None` = all.
    pub workspaces: Option<Vec<WorkspaceId>>,
    /// Creates the panel state when it is first opened.
    pub create: fn() -> Box<dyn Panel>,
}

/// A dockable panel. Lives on the UI thread; all document mutation goes through
/// [`AppCx::submit`].
pub trait Panel: 'static {
    /// Draws the panel.
    fn ui(&mut self, ui: &mut egui::Ui, cx: &mut AppCx<'_>);
    /// The document events of the frame (active document only).
    fn on_doc_events(&mut self, _events: &[amg_doc::DocEvent], _env: &CmdEnv<'_>) {}
}
