//! `DockHost`: the facade over the docking crate (ADR-005 amendment, note §1).
//!
//! Only this module names `egui_dock`. The shell asks for what it needs: open a tab in a slot,
//! focus or close it, serialise the layout and draw it. Switching to `egui_tiles` (the
//! fallback) would only change this file.

use egui_dock::{DockArea, DockState, NodeIndex, Style, TabViewer};

use crate::module::PanelId;
use crate::panel::DockSlot;
use crate::registry::UiRegistry;

/// A tab in the dock area.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub enum DockTab {
    /// The 3D viewport of the active document.
    Viewport,
    /// A module panel.
    Panel(PanelId),
}

/// Serialised form of [`DockTab`] (panel ids as strings).
#[derive(Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
enum SavedTab {
    Viewport,
    Panel(String),
}

/// What the shell draws inside each dock tab.
pub trait DockContent {
    /// Tab title.
    fn title(&mut self, tab: DockTab) -> egui::WidgetText;
    /// Tab body.
    fn ui(&mut self, ui: &mut egui::Ui, tab: DockTab);
}

/// The dock area of one workspace.
pub struct DockHost {
    state: DockState<DockTab>,
}

impl std::fmt::Debug for DockHost {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("DockHost")
            .field("tabs", &self.tabs())
            .finish()
    }
}

impl Default for DockHost {
    fn default() -> Self {
        DockHost::new()
    }
}

/// Error reading a saved layout.
#[derive(Debug, thiserror::Error)]
pub enum LayoutError {
    /// Not valid layout JSON.
    #[error("invalid dock layout: {0}")]
    Json(#[from] serde_json::Error),
}

impl DockHost {
    /// A dock area holding only the viewport tab.
    pub fn new() -> DockHost {
        DockHost {
            state: DockState::new(vec![DockTab::Viewport]),
        }
    }

    /// Every tab, in tree order.
    pub fn tabs(&self) -> Vec<DockTab> {
        self.state.iter_all_tabs().map(|(_, t)| *t).collect()
    }

    /// True if the tab is open.
    pub fn contains(&self, tab: DockTab) -> bool {
        self.state.find_tab(&tab).is_some()
    }

    /// Opens a panel in a slot; focuses it if it is already open.
    pub fn open(&mut self, panel: PanelId, slot: DockSlot) {
        let tab = DockTab::Panel(panel);
        if self.focus(tab) {
            return;
        }
        let viewport_node = self
            .state
            .main_surface()
            .find_tab(&DockTab::Viewport)
            .map_or(NodeIndex::root(), |(n, _)| n);
        let surface = self.state.main_surface_mut();
        match slot {
            DockSlot::CenterTab => {
                surface.set_focused_node(viewport_node);
                surface.push_to_focused_leaf(tab);
            }
            DockSlot::Left => {
                surface.split_left(viewport_node, 0.25, vec![tab]);
            }
            DockSlot::Right => {
                surface.split_right(viewport_node, 0.75, vec![tab]);
            }
            DockSlot::Bottom => {
                surface.split_below(viewport_node, 0.7, vec![tab]);
            }
            DockSlot::Floating => {
                self.state.add_window(vec![tab]);
            }
        }
    }

    /// Makes an open tab the active one of its leaf. Returns false if it is not open.
    pub fn focus(&mut self, tab: DockTab) -> bool {
        match self.state.find_tab(&tab) {
            Some(path) => {
                let _ = self.state.set_active_tab(path);
                true
            }
            None => false,
        }
    }

    /// Closes a tab. The viewport tab cannot be closed. Returns true if a tab was removed.
    pub fn close(&mut self, tab: DockTab) -> bool {
        if tab == DockTab::Viewport {
            return false;
        }
        match self.state.find_tab(&tab) {
            Some(path) => self.state.remove_tab(path).is_some(),
            None => false,
        }
    }

    /// Keeps only the tabs for which `keep` is true (the viewport always stays).
    pub fn retain(&mut self, mut keep: impl FnMut(DockTab) -> bool) {
        self.state
            .retain_tabs(|t| *t == DockTab::Viewport || keep(*t));
        if !self.contains(DockTab::Viewport) {
            self.state
                .main_surface_mut()
                .push_to_first_leaf(DockTab::Viewport);
        }
    }

    /// The layout as JSON.
    pub fn to_json(&self) -> serde_json::Value {
        let saved = self.state.map_tabs(|t| match t {
            DockTab::Viewport => SavedTab::Viewport,
            DockTab::Panel(p) => SavedTab::Panel(p.0.to_owned()),
        });
        // A dock state holds only strings, numbers and enums; serialising cannot fail.
        let mut json = serde_json::to_value(&saved).unwrap_or(serde_json::Value::Null);
        zero_non_finite_points(&mut json);
        json
    }

    /// Restores a layout. Panels the registry no longer knows are dropped; a missing viewport
    /// tab is added back.
    ///
    /// # Errors
    /// [`LayoutError::Json`] if `json` is not a saved layout.
    pub fn from_json(json: &serde_json::Value, reg: &UiRegistry) -> Result<DockHost, LayoutError> {
        let saved: DockState<SavedTab> = serde_json::from_value(json.clone())?;
        let state = saved.filter_map_tabs(|t| match t {
            SavedTab::Viewport => Some(DockTab::Viewport),
            SavedTab::Panel(s) => reg.panel_by_str(s).map(DockTab::Panel),
        });
        let mut host = DockHost { state };
        if !host.contains(DockTab::Viewport) {
            host.state
                .main_surface_mut()
                .push_to_first_leaf(DockTab::Viewport);
        }
        Ok(host)
    }

    /// Draws the dock area into `ui`.
    pub fn show(&mut self, ui: &mut egui::Ui, content: &mut dyn DockContent) {
        let style = Style::from_egui(ui.style());
        let mut viewer = Viewer { content };
        DockArea::new(&mut self.state)
            .id(egui::Id::new("amg_dock"))
            .style(style)
            .show_add_buttons(false)
            .show_leaf_close_all_buttons(false)
            .show_leaf_collapse_buttons(false)
            .show_inside(ui, &mut viewer);
    }

    /// Number of floating windows (tests).
    pub fn window_count(&self) -> usize {
        self.state
            .iter_surfaces_indexed()
            .filter(|(i, s)| !i.is_main() && matches!(s, egui_dock::Surface::Window(..)))
            .count()
    }
}

/// Rects of nodes never laid out are `Rect::NOTHING` (±∞), which JSON writes as `null` and
/// cannot read back. Such coordinates become 0; egui_dock recomputes every rect on the next
/// layout. Only `{x, y}` objects are touched, so `Option` nulls stay as they are.
fn zero_non_finite_points(v: &mut serde_json::Value) {
    match v {
        serde_json::Value::Object(map) => {
            let is_point = map.len() == 2 && map.contains_key("x") && map.contains_key("y");
            for (k, child) in map.iter_mut() {
                if is_point && (k == "x" || k == "y") && child.is_null() {
                    *child = serde_json::Value::from(0.0);
                } else {
                    zero_non_finite_points(child);
                }
            }
        }
        serde_json::Value::Array(items) => items.iter_mut().for_each(zero_non_finite_points),
        _ => {}
    }
}

struct Viewer<'a> {
    content: &'a mut dyn DockContent,
}

impl TabViewer for Viewer<'_> {
    type Tab = DockTab;

    fn id(&mut self, tab: &mut DockTab) -> egui::Id {
        egui::Id::new(("amg_dock_tab", *tab))
    }

    fn title(&mut self, tab: &mut DockTab) -> egui::WidgetText {
        self.content.title(*tab)
    }

    fn ui(&mut self, ui: &mut egui::Ui, tab: &mut DockTab) {
        self.content.ui(ui, *tab);
    }

    fn is_closeable(&self, tab: &DockTab) -> bool {
        *tab != DockTab::Viewport
    }

    fn allowed_in_windows(&self, tab: &mut DockTab) -> bool {
        *tab != DockTab::Viewport
    }

    fn clear_background(&self, tab: &DockTab) -> bool {
        *tab != DockTab::Viewport
    }

    fn scroll_bars(&self, tab: &DockTab) -> [bool; 2] {
        match tab {
            DockTab::Viewport => [false, false],
            DockTab::Panel(_) => [true, true],
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::module::{IconId, Registrar, UiModule};
    use crate::panel::{Panel, PanelDef};

    struct Empty;
    impl Panel for Empty {
        fn ui(&mut self, _ui: &mut egui::Ui, _cx: &mut crate::app::AppCx<'_>) {}
    }

    fn panels(r: &mut Registrar<'_>) {
        for (id, slot) in [
            ("t.left", DockSlot::Left),
            ("t.center", DockSlot::CenterTab),
            ("t.bottom", DockSlot::Bottom),
        ] {
            r.panel(PanelDef {
                id: PanelId(id),
                title: id,
                icon: IconId::NONE,
                slot,
                workspaces: None,
                create: || Box::new(Empty),
            });
        }
    }

    #[test]
    fn open_focus_close_and_round_trip() {
        let m = UiModule {
            id: "t",
            register: panels,
        };
        let reg = UiRegistry::from_modules([&m]);
        let mut dock = DockHost::new();
        dock.open(PanelId("t.left"), DockSlot::Left);
        dock.open(PanelId("t.center"), DockSlot::CenterTab);
        dock.open(PanelId("t.bottom"), DockSlot::Bottom);
        dock.open(PanelId("t.left"), DockSlot::Left); // already open: focus only
        assert_eq!(dock.tabs().len(), 4);
        assert!(!dock.close(DockTab::Viewport));

        let json = dock.to_json();
        let back = DockHost::from_json(&json, &reg).unwrap();
        assert_eq!(back.tabs(), dock.tabs());

        // A panel that no longer exists is dropped on load.
        let empty = UiRegistry::default();
        let pruned = DockHost::from_json(&json, &empty).unwrap();
        assert_eq!(pruned.tabs(), vec![DockTab::Viewport]);

        assert!(dock.close(DockTab::Panel(PanelId("t.center"))));
        assert!(!dock.contains(DockTab::Panel(PanelId("t.center"))));
        dock.retain(|t| t != DockTab::Panel(PanelId("t.bottom")));
        assert!(dock.contains(DockTab::Viewport));
        assert!(dock.contains(DockTab::Panel(PanelId("t.left"))));
        assert!(!dock.contains(DockTab::Panel(PanelId("t.bottom"))));
        dock.open(PanelId("t.center"), DockSlot::Floating);
        assert_eq!(dock.window_count(), 1);
        assert!(dock.close(DockTab::Panel(PanelId("t.center"))));
        assert_eq!(dock.window_count(), 0);
    }
}
