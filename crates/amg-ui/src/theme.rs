//! Light/dark themes: [`ThemeTokens`] drive egui's visuals and the viewport (spec §3).

use egui::Color32;

/// The user's theme choice.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
pub enum Theme {
    /// Light.
    Light,
    /// Dark.
    Dark,
    /// Follow the OS (dark when the OS does not say).
    #[default]
    System,
}

impl Theme {
    /// All choices, in menu order.
    pub const ALL: [Theme; 3] = [Theme::System, Theme::Light, Theme::Dark];

    /// Menu label.
    pub fn label(self) -> &'static str {
        match self {
            Theme::Light => "Light",
            Theme::Dark => "Dark",
            Theme::System => "System",
        }
    }

    /// The concrete egui theme, given the OS theme if known.
    pub fn resolve(self, system: Option<egui::Theme>) -> egui::Theme {
        match self {
            Theme::Light => egui::Theme::Light,
            Theme::Dark => egui::Theme::Dark,
            Theme::System => system.unwrap_or(egui::Theme::Dark),
        }
    }
}

/// Colours shared by the panels and the viewport (S10 highlight, overlay and background).
#[derive(Clone, Debug, PartialEq)]
pub struct ThemeTokens {
    /// Accent (selection in widgets, active tab).
    pub accent: Color32,
    /// Hover / pre-selection highlight.
    pub hover: Color32,
    /// Selection highlight.
    pub selected: Color32,
    /// Preview geometry.
    pub preview: Color32,
    /// Errors.
    pub error: Color32,
    /// Warnings.
    pub warning: Color32,
    /// Sketch DOF: under-constrained.
    pub dof_under: Color32,
    /// Sketch DOF: fully constrained.
    pub dof_full: Color32,
    /// Sketch DOF: over-constrained.
    pub dof_over: Color32,
    /// Construction geometry.
    pub construction: Color32,
    /// Grid lines.
    pub grid: Color32,
    /// Viewport background, top of the gradient.
    pub background_top: Color32,
    /// Viewport background, bottom of the gradient.
    pub background_bottom: Color32,
}

impl ThemeTokens {
    /// Tokens of a concrete theme.
    pub fn for_theme(theme: egui::Theme) -> ThemeTokens {
        match theme {
            egui::Theme::Dark => ThemeTokens::dark(),
            egui::Theme::Light => ThemeTokens::light(),
        }
    }

    /// Dark tokens.
    pub fn dark() -> ThemeTokens {
        ThemeTokens {
            accent: Color32::from_rgb(6, 150, 215),
            hover: Color32::from_rgb(120, 196, 255),
            selected: Color32::from_rgb(20, 130, 255),
            preview: Color32::from_rgba_unmultiplied(110, 200, 120, 160),
            error: Color32::from_rgb(232, 72, 72),
            warning: Color32::from_rgb(240, 180, 40),
            dof_under: Color32::from_rgb(70, 150, 255),
            dof_full: Color32::from_rgb(230, 230, 230),
            dof_over: Color32::from_rgb(232, 64, 64),
            construction: Color32::from_rgb(240, 150, 40),
            grid: Color32::from_rgb(88, 92, 100),
            background_top: Color32::from_rgb(64, 68, 76),
            background_bottom: Color32::from_rgb(30, 32, 36),
        }
    }

    /// Light tokens (Fusion's default white-to-grey viewport).
    pub fn light() -> ThemeTokens {
        ThemeTokens {
            accent: Color32::from_rgb(0, 120, 200),
            hover: Color32::from_rgb(90, 170, 255),
            selected: Color32::from_rgb(0, 110, 230),
            preview: Color32::from_rgba_unmultiplied(70, 170, 90, 160),
            error: Color32::from_rgb(200, 40, 40),
            warning: Color32::from_rgb(200, 140, 0),
            dof_under: Color32::from_rgb(30, 110, 230),
            dof_full: Color32::from_rgb(20, 20, 20),
            dof_over: Color32::from_rgb(210, 30, 30),
            construction: Color32::from_rgb(220, 120, 20),
            grid: Color32::from_rgb(196, 200, 206),
            background_top: Color32::from_rgb(255, 255, 255),
            background_bottom: Color32::from_rgb(196, 204, 214),
        }
    }

    /// The viewport background as one linear RGB colour (S10 has only `Background::Solid` so
    /// far): the gradient's midpoint.
    pub fn background_linear(&self) -> [f32; 3] {
        let a = egui::Rgba::from(self.background_top);
        let b = egui::Rgba::from(self.background_bottom);
        [
            0.5 * (a.r() + b.r()),
            0.5 * (a.g() + b.g()),
            0.5 * (a.b() + b.b()),
        ]
    }

    /// egui visuals for a concrete theme, with these tokens applied.
    pub fn visuals(&self, theme: egui::Theme) -> egui::Visuals {
        let mut v = match theme {
            egui::Theme::Dark => egui::Visuals::dark(),
            egui::Theme::Light => egui::Visuals::light(),
        };
        v.selection.bg_fill = self.accent;
        // Text on the accent (selected tab, selected label) must stay readable in both themes.
        v.selection.stroke.color = Color32::WHITE;
        v.hyperlink_color = self.accent;
        v.error_fg_color = self.error;
        v.warn_fg_color = self.warning;
        v.window_corner_radius = egui::CornerRadius::same(4);
        v.menu_corner_radius = egui::CornerRadius::same(3);
        match theme {
            egui::Theme::Dark => {
                v.panel_fill = Color32::from_rgb(38, 39, 42);
                v.window_fill = Color32::from_rgb(46, 47, 51);
                v.extreme_bg_color = Color32::from_rgb(26, 27, 29);
                v.faint_bg_color = Color32::from_rgb(44, 45, 49);
            }
            egui::Theme::Light => {
                v.panel_fill = Color32::from_rgb(243, 243, 244);
                v.window_fill = Color32::from_rgb(252, 252, 252);
                v.extreme_bg_color = Color32::from_rgb(255, 255, 255);
                v.faint_bg_color = Color32::from_rgb(236, 237, 239);
            }
        }
        v
    }
}

/// Applies a theme to egui only when the resolved theme changed, so an idle app never sets
/// its style (and repaints) again. Returns the concrete theme and its tokens.
#[derive(Debug, Default)]
pub(crate) struct ThemeState {
    applied: Option<egui::Theme>,
}

impl ThemeState {
    pub(crate) fn apply(&mut self, ctx: &egui::Context, choice: Theme) -> Option<ThemeTokens> {
        let theme = choice.resolve(ctx.system_theme());
        if self.applied == Some(theme) {
            return None;
        }
        self.applied = Some(theme);
        let tokens = ThemeTokens::for_theme(theme);
        ctx.set_visuals_of(theme, tokens.visuals(theme));
        ctx.set_theme(theme);
        Some(tokens)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn system_falls_back_to_dark() {
        assert_eq!(Theme::System.resolve(None), egui::Theme::Dark);
        assert_eq!(
            Theme::System.resolve(Some(egui::Theme::Light)),
            egui::Theme::Light
        );
        assert_eq!(
            Theme::Light.resolve(Some(egui::Theme::Dark)),
            egui::Theme::Light
        );
    }

    #[test]
    fn tokens_reach_the_visuals() {
        for theme in [egui::Theme::Light, egui::Theme::Dark] {
            let t = ThemeTokens::for_theme(theme);
            let v = t.visuals(theme);
            assert_eq!(v.selection.bg_fill, t.accent);
            assert_eq!(v.dark_mode, theme == egui::Theme::Dark);
        }
        let light = ThemeTokens::light().background_linear();
        let dark = ThemeTokens::dark().background_linear();
        assert!(light[0] > dark[0]);
    }

    #[test]
    fn applies_only_on_change() {
        let ctx = egui::Context::default();
        let mut s = ThemeState::default();
        assert!(s.apply(&ctx, Theme::Light).is_some());
        assert!(s.apply(&ctx, Theme::Light).is_none());
        assert!(!ctx.global_style().visuals.dark_mode);
        assert!(s.apply(&ctx, Theme::Dark).is_some());
        assert!(ctx.global_style().visuals.dark_mode);
    }
}
