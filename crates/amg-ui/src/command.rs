//! The command registry types (spec S11 §4, Interfaces §A `amg_ui::command`).
//!
//! A [`CommandDef`] feeds the toolbar and menus ([`Placement`]), S-key search, shortcuts and
//! (later) the marking menu. `amg_doc::Command` (a document mutation) is a different thing and
//! is imported as `DocCommand` elsewhere.
//!
//! `CmdAction::StartTool` joins with the `Tool` trait in UI-03 (additive).

use std::borrow::Cow;
use std::fmt;
use std::sync::Arc;

use crate::app::{AppCx, CmdEnv};
use crate::module::{GroupId, IconId};
use crate::shortcut::Shortcut;

/// A command id, module-prefixed and unique (`"mesh.remesh"`, `"script.my_addin"`).
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct CmdId(pub &'static str);

impl CmdId {
    /// The id string.
    pub fn as_str(self) -> &'static str {
        self.0
    }
}

impl fmt::Display for CmdId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.0)
    }
}

/// Where a command is shown besides S-key search (which lists every command).
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Placement {
    /// In a toolbar group; `pinned` also shows a top-level button beside the group dropdown.
    Toolbar {
        /// The group.
        group: GroupId,
        /// Also a top-level button.
        pinned: bool,
        /// Sort key inside the group (then the id).
        order: i32,
    },
    /// In a menu: `"File"`, `"File/Export"`, `"Edit"`. The first segment is a top-level menu,
    /// the others are submenus.
    Menu {
        /// Slash-separated menu path.
        path: &'static str,
        /// Sort key inside the menu level (then the id).
        order: i32,
    },
    /// Only found through search and shortcuts.
    SearchOnly,
}

/// Whether a command can run now.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Enablement {
    /// Can run.
    Enabled,
    /// Cannot run; the reason is the tooltip.
    Disabled(Cow<'static, str>),
}

impl Enablement {
    /// `Disabled` with a reason.
    pub fn disabled(reason: impl Into<Cow<'static, str>>) -> Enablement {
        Enablement::Disabled(reason.into())
    }

    /// True for [`Enablement::Enabled`].
    pub fn is_enabled(&self) -> bool {
        matches!(self, Enablement::Enabled)
    }

    /// The reason a command is disabled.
    pub fn reason(&self) -> Option<&str> {
        match self {
            Enablement::Enabled => None,
            Enablement::Disabled(r) => Some(r),
        }
    }
}

/// Dynamic enablement closure.
pub type EnableFn = Arc<dyn Fn(&CmdEnv<'_>) -> Enablement + Send + Sync>;
/// Dynamic action closure (scripting).
pub type ActionFn = Arc<dyn Fn(&mut AppCx<'_>) + Send + Sync>;

/// How enablement is decided. Evaluated lazily: only for visible buttons and search results.
#[derive(Clone)]
pub enum Enabler {
    /// Always enabled.
    Always,
    /// A plain function of the environment.
    When(fn(&CmdEnv<'_>) -> Enablement),
    /// A closure (scripting).
    Dyn(EnableFn),
}

impl Enabler {
    /// Evaluates the enabler.
    pub fn eval(&self, env: &CmdEnv<'_>) -> Enablement {
        match self {
            Enabler::Always => Enablement::Enabled,
            Enabler::When(f) => f(env),
            Enabler::Dyn(f) => f(env),
        }
    }
}

impl fmt::Debug for Enabler {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Enabler::Always => "Always",
            Enabler::When(_) => "When(..)",
            Enabler::Dyn(_) => "Dyn(..)",
        })
    }
}

/// What a command does when it runs. Actions run at the end of the frame, so they never see a
/// half-drawn UI and never block it.
#[derive(Clone)]
pub enum CmdAction {
    /// Run a function.
    Run(fn(&mut AppCx<'_>)),
    /// A checkable command: `get` reads the state, `set` receives the new one.
    Toggle {
        /// Current state.
        get: fn(&CmdEnv<'_>) -> bool,
        /// Applies the new state.
        set: fn(&mut AppCx<'_>, bool),
    },
    /// A closure (scripting, SCR-04).
    Dyn(ActionFn),
}

impl fmt::Debug for CmdAction {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            CmdAction::Run(_) => "Run(..)",
            CmdAction::Toggle { .. } => "Toggle{..}",
            CmdAction::Dyn(_) => "Dyn(..)",
        })
    }
}

/// A UI command.
#[derive(Clone, Debug)]
pub struct CommandDef {
    /// Unique id.
    pub id: CmdId,
    /// Label in toolbar, menus and search.
    pub label: Cow<'static, str>,
    /// Tooltip (empty: none).
    pub tooltip: Cow<'static, str>,
    /// Icon.
    pub icon: IconId,
    /// Where the command is placed.
    pub placements: Vec<Placement>,
    /// Default shortcut (the user's `shortcuts.toml` can replace it).
    pub shortcut: Option<Shortcut>,
    /// Further default shortcuts (e.g. `Ctrl+Shift+Z` for redo). Additive to the spec.
    pub alt_shortcuts: Vec<Shortcut>,
    /// Extra S-key search terms.
    pub keywords: &'static [&'static str],
    /// When the command can run.
    pub enabled: Enabler,
    /// What it does.
    pub action: CmdAction,
}

impl CommandDef {
    /// A command that is always enabled, has no placement, shortcut or keywords and runs
    /// `action`. Fill the rest with the `with_*` methods.
    pub fn new(
        id: &'static str,
        label: impl Into<Cow<'static, str>>,
        action: CmdAction,
    ) -> CommandDef {
        CommandDef {
            id: CmdId(id),
            label: label.into(),
            tooltip: Cow::Borrowed(""),
            icon: IconId::NONE,
            placements: Vec::new(),
            shortcut: None,
            alt_shortcuts: Vec::new(),
            keywords: &[],
            enabled: Enabler::Always,
            action,
        }
    }

    /// Sets the tooltip.
    #[must_use]
    pub fn with_tooltip(mut self, tooltip: impl Into<Cow<'static, str>>) -> CommandDef {
        self.tooltip = tooltip.into();
        self
    }

    /// Sets the icon.
    #[must_use]
    pub fn with_icon(mut self, icon: IconId) -> CommandDef {
        self.icon = icon;
        self
    }

    /// Adds a placement.
    #[must_use]
    pub fn placed(mut self, p: Placement) -> CommandDef {
        self.placements.push(p);
        self
    }

    /// Places the command in a toolbar group.
    #[must_use]
    pub fn in_group(self, group: GroupId, order: i32) -> CommandDef {
        self.placed(Placement::Toolbar {
            group,
            pinned: false,
            order,
        })
    }

    /// Places the command in a menu (`"File/Export"`).
    #[must_use]
    pub fn in_menu(self, path: &'static str, order: i32) -> CommandDef {
        self.placed(Placement::Menu { path, order })
    }

    /// Sets the default shortcut from its text form (`"Ctrl+S"`, `"sketch:L"`).
    ///
    /// A malformed literal is a programming error: it panics in debug builds and is logged and
    /// ignored in release builds.
    #[must_use]
    pub fn with_shortcut(mut self, text: &str) -> CommandDef {
        match Shortcut::parse(text) {
            Ok(s) => self.shortcut = Some(s),
            Err(e) => bad_literal(self.id, text, &e),
        }
        self
    }

    /// Adds a further default shortcut from its text form.
    #[must_use]
    pub fn with_alt_shortcut(mut self, text: &str) -> CommandDef {
        match Shortcut::parse(text) {
            Ok(s) => self.alt_shortcuts.push(s),
            Err(e) => bad_literal(self.id, text, &e),
        }
        self
    }

    /// Sets the search keywords.
    #[must_use]
    pub fn with_keywords(mut self, keywords: &'static [&'static str]) -> CommandDef {
        self.keywords = keywords;
        self
    }

    /// Sets the enabler.
    #[must_use]
    pub fn with_enabler(mut self, enabled: Enabler) -> CommandDef {
        self.enabled = enabled;
        self
    }

    /// Evaluates [`CommandDef::enabled`].
    pub fn enablement(&self, env: &CmdEnv<'_>) -> Enablement {
        self.enabled.eval(env)
    }

    /// True for a [`CmdAction::Toggle`] command.
    pub fn is_toggle(&self) -> bool {
        matches!(self.action, CmdAction::Toggle { .. })
    }

    /// The checked state of a toggle command (`false` for others).
    pub fn checked(&self, env: &CmdEnv<'_>) -> bool {
        match &self.action {
            CmdAction::Toggle { get, .. } => get(env),
            _ => false,
        }
    }

    /// The default shortcuts, the primary one first.
    pub fn default_shortcuts(&self) -> impl Iterator<Item = &Shortcut> + '_ {
        self.shortcut.iter().chain(self.alt_shortcuts.iter())
    }
}

fn bad_literal(id: CmdId, text: &str, err: &dyn fmt::Display) {
    let msg = format!("command `{id}`: bad shortcut literal `{text}`: {err}");
    if cfg!(debug_assertions) {
        panic!("{msg}");
    }
    tracing::error!("{msg}; ignored");
}

/// Why the registry refused a runtime registration.
#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum RegistryError {
    /// The id is taken.
    #[error("command id `{0}` already registered")]
    Duplicate(&'static str),
    /// Runtime ids must be in the `script.` namespace.
    #[error("dynamic command ids must start with `script.`: `{0}`")]
    BadPrefix(&'static str),
}

/// The namespace of runtime-registered (scripting) commands.
pub const DYNAMIC_PREFIX: &str = "script.";
