//! Shortcuts, scopes and the `ShortcutMap` (spec S11 §4, Interfaces §A `amg_ui::shortcut`).
//!
//! `ShortcutMap` = the registered defaults overlaid with the user's `shortcuts.toml`:
//!
//! ```toml
//! [shortcuts]
//! "mesh.remesh" = "Ctrl+Alt+R"   # replaces the default
//! "sketch.line" = "sketch:K"     # scope prefix: sketch:, viewport:, global:
//! "edit.redo"   = ""             # unbinds every default
//! ```
//!
//! Scopes are `Sketch` > `Viewport` > `Global`. A conflict inside one scope is recorded (the
//! preferences page, UI-11, lists them) and the later binding is ignored; user overrides are
//! bound before the defaults, so they win.

use std::collections::{BTreeMap, HashMap};
use std::fmt;

use serde::{Deserialize, Deserializer, Serialize, Serializer};

use crate::command::CmdId;
use crate::registry::UiRegistry;

/// Where a shortcut is active. Higher scopes win.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub enum ShortcutScope {
    /// Everywhere.
    Global,
    /// While the pointer is over the viewport.
    Viewport,
    /// While a sketch is being edited.
    Sketch,
}

impl ShortcutScope {
    /// Every scope, highest priority first.
    pub const BY_PRIORITY: [ShortcutScope; 3] = [
        ShortcutScope::Sketch,
        ShortcutScope::Viewport,
        ShortcutScope::Global,
    ];

    fn prefix(self) -> &'static str {
        match self {
            ShortcutScope::Global => "global",
            ShortcutScope::Viewport => "viewport",
            ShortcutScope::Sketch => "sketch",
        }
    }
}

/// A key with modifiers in a scope.
///
/// Text form: `"Ctrl+Shift+S"`, `"viewport:E"`, `"sketch:Shift+L"`. `Ctrl` is the platform
/// command key (Cmd on macOS). A missing scope prefix means [`ShortcutScope::Global`].
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct Shortcut {
    /// Modifiers (exact match).
    pub mods: egui::Modifiers,
    /// The key.
    pub key: egui::Key,
    /// Where it is active.
    pub scope: ShortcutScope,
}

/// A malformed shortcut text.
#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum ShortcutParseError {
    /// Nothing to parse.
    #[error("empty shortcut")]
    Empty,
    /// Unknown scope prefix.
    #[error("unknown scope `{0}` (expected global, viewport or sketch)")]
    Scope(String),
    /// Unknown modifier.
    #[error("unknown modifier `{0}`")]
    Modifier(String),
    /// Unknown key name.
    #[error("unknown key `{0}`")]
    Key(String),
}

impl Shortcut {
    /// A global shortcut.
    pub fn new(mods: egui::Modifiers, key: egui::Key) -> Shortcut {
        Shortcut {
            mods,
            key,
            scope: ShortcutScope::Global,
        }
    }

    /// Parses the text form.
    ///
    /// # Errors
    /// [`ShortcutParseError`] for an unknown scope, modifier or key.
    pub fn parse(s: &str) -> Result<Shortcut, ShortcutParseError> {
        let s = s.trim();
        if s.is_empty() {
            return Err(ShortcutParseError::Empty);
        }
        let (scope, rest) = match s.split_once(':') {
            // A lone ":" is the colon key; only a non-empty prefix is a scope.
            Some((p, rest)) if !p.is_empty() && !rest.is_empty() => {
                let scope = match p.to_ascii_lowercase().as_str() {
                    "global" => ShortcutScope::Global,
                    "viewport" => ShortcutScope::Viewport,
                    "sketch" => ShortcutScope::Sketch,
                    other => return Err(ShortcutParseError::Scope(other.to_owned())),
                };
                (scope, rest)
            }
            _ => (ShortcutScope::Global, s),
        };
        let mut mods = egui::Modifiers::NONE;
        let mut parts: Vec<&str> = rest.split('+').map(str::trim).collect();
        // "Ctrl++" (the plus key) leaves two empty trailing parts.
        let key_text = if rest.ends_with('+') {
            while parts.last() == Some(&"") {
                parts.pop();
            }
            "+".to_owned()
        } else {
            parts.pop().unwrap_or_default().to_owned()
        };
        for m in parts {
            match m.to_ascii_lowercase().as_str() {
                "ctrl" | "control" | "cmd" | "command" => mods |= egui::Modifiers::COMMAND,
                "shift" => mods |= egui::Modifiers::SHIFT,
                "alt" | "option" => mods |= egui::Modifiers::ALT,
                other => return Err(ShortcutParseError::Modifier(other.to_owned())),
            }
        }
        let key = parse_key(&key_text).ok_or(ShortcutParseError::Key(key_text))?;
        Ok(Shortcut { mods, key, scope })
    }

    /// The text without the scope prefix, as shown next to menu items (`"Ctrl+Shift+Z"`).
    pub fn display(&self) -> String {
        let mut out = String::new();
        if self.mods.command || self.mods.ctrl {
            out.push_str("Ctrl+");
        }
        if self.mods.alt {
            out.push_str("Alt+");
        }
        if self.mods.shift {
            out.push_str("Shift+");
        }
        out.push_str(self.key.name());
        out
    }

    /// The parseable text form, with the scope prefix unless the scope is global.
    pub fn to_text(&self) -> String {
        match self.scope {
            ShortcutScope::Global => self.display(),
            s => format!("{}:{}", s.prefix(), self.display()),
        }
    }

    /// True if `mods` (the live modifier state) press exactly this shortcut's modifiers.
    pub fn mods_match(&self, mods: egui::Modifiers) -> bool {
        mods.matches_exact(self.mods)
    }

    /// True if the shortcut needs Ctrl/Cmd or Alt, so it is not a text-entry key.
    pub fn has_command_modifier(&self) -> bool {
        self.mods.command || self.mods.ctrl || self.mods.alt || self.mods.mac_cmd
    }
}

impl fmt::Display for Shortcut {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.to_text())
    }
}

fn parse_key(text: &str) -> Option<egui::Key> {
    if let Some(k) = egui::Key::from_name(text) {
        return Some(k);
    }
    // Letters are case-insensitive in the file: "l" and "L" are the same key.
    let mut chars = text.chars();
    if let (Some(c), None) = (chars.next(), chars.next())
        && c.is_ascii_alphabetic()
    {
        return egui::Key::from_name(&c.to_ascii_uppercase().to_string());
    }
    egui::Key::from_name(&capitalised(text))
}

fn capitalised(s: &str) -> String {
    let mut c = s.chars();
    match c.next() {
        Some(f) => f.to_ascii_uppercase().to_string() + c.as_str(),
        None => String::new(),
    }
}

impl Serialize for Shortcut {
    fn serialize<S: Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        s.serialize_str(&self.to_text())
    }
}

impl<'de> Deserialize<'de> for Shortcut {
    fn deserialize<D: Deserializer<'de>>(d: D) -> Result<Shortcut, D::Error> {
        let text = String::deserialize(d)?;
        Shortcut::parse(&text).map_err(serde::de::Error::custom)
    }
}

/// The scopes active now, highest priority first: Sketch only while a sketch is being edited,
/// Viewport only while the pointer is over the viewport, Global always.
pub fn active_scopes(sketch_active: bool, viewport_hovered: bool) -> Vec<ShortcutScope> {
    let mut v = Vec::with_capacity(3);
    if sketch_active {
        v.push(ShortcutScope::Sketch);
    }
    if viewport_hovered {
        v.push(ShortcutScope::Viewport);
    }
    v.push(ShortcutScope::Global);
    v
}

/// The user's `shortcuts.toml`: command id → replacement text (`""` unbinds).
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct ShortcutOverrides {
    entries: BTreeMap<String, String>,
}

/// A `shortcuts.toml` that cannot be used.
#[derive(Debug, thiserror::Error)]
pub enum ShortcutFileError {
    /// Not valid TOML or not the expected shape.
    #[error("shortcuts.toml: {0}")]
    Toml(#[from] toml::de::Error),
    /// A binding with an unparseable text.
    #[error("shortcuts.toml: `{id}` = `{text}`: {source}")]
    Binding {
        /// The command id.
        id: String,
        /// The bad text.
        text: String,
        /// Why.
        source: ShortcutParseError,
    },
}

#[derive(Deserialize, Default)]
struct OverridesFile {
    #[serde(default)]
    shortcuts: BTreeMap<String, String>,
}

impl ShortcutOverrides {
    /// No overrides.
    pub fn none() -> ShortcutOverrides {
        ShortcutOverrides::default()
    }

    /// Builds overrides from pairs (tests and the preferences editor).
    pub fn from_pairs<'a>(pairs: impl IntoIterator<Item = (&'a str, &'a str)>) -> Self {
        ShortcutOverrides {
            entries: pairs
                .into_iter()
                .map(|(k, v)| (k.to_owned(), v.to_owned()))
                .collect(),
        }
    }

    /// Parses `shortcuts.toml`.
    ///
    /// # Errors
    /// [`ShortcutFileError`] for malformed TOML or a binding whose text does not parse; the
    /// whole file is rejected so a typo never silently drops one binding.
    pub fn parse(text: &str) -> Result<ShortcutOverrides, ShortcutFileError> {
        let file: OverridesFile = toml::from_str(text)?;
        for (id, t) in &file.shortcuts {
            if !t.trim().is_empty()
                && let Err(source) = Shortcut::parse(t)
            {
                return Err(ShortcutFileError::Binding {
                    id: id.clone(),
                    text: t.clone(),
                    source,
                });
            }
        }
        Ok(ShortcutOverrides {
            entries: file.shortcuts,
        })
    }

    /// Serialises to `shortcuts.toml`.
    pub fn to_toml(&self) -> String {
        let mut out = String::from("[shortcuts]\n");
        for (id, t) in &self.entries {
            out.push_str(&format!("{id:?} = {t:?}\n"));
        }
        out
    }

    /// Number of overrides.
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// True without overrides.
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }
}

/// Two commands bound to the same key in one scope; the second binding is ignored.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ShortcutConflict {
    /// The scope.
    pub scope: ShortcutScope,
    /// The shortcut text (without scope).
    pub shortcut: String,
    /// The command that owns the binding.
    pub kept: CmdId,
    /// The command whose binding was ignored.
    pub ignored: CmdId,
}

#[derive(Clone, Debug)]
struct Binding {
    mods: egui::Modifiers,
    cmd: CmdId,
}

/// The effective bindings.
#[derive(Clone, Debug, Default)]
pub struct ShortcutMap {
    by_key: HashMap<(ShortcutScope, egui::Key), Vec<Binding>>,
    by_cmd: BTreeMap<CmdId, Vec<Shortcut>>,
    conflicts: Vec<ShortcutConflict>,
    unknown: Vec<String>,
    generation: u64,
}

impl ShortcutMap {
    /// Overlays `overrides` on the defaults registered in `reg`.
    ///
    /// Overrides are bound first (they win a conflict), then the defaults of every command in id
    /// order. An override for an unknown command is kept in [`Self::unknown_ids`] (a command
    /// may be registered later, e.g. by an add-in).
    pub fn build(reg: &UiRegistry, overrides: &ShortcutOverrides) -> ShortcutMap {
        let mut map = ShortcutMap {
            generation: reg.generation(),
            ..ShortcutMap::default()
        };
        let mut overridden: Vec<CmdId> = Vec::new();
        for (id, text) in &overrides.entries {
            let Some(def) = reg.commands().find(|c| c.id.0 == id) else {
                map.unknown.push(id.clone());
                continue;
            };
            overridden.push(def.id);
            let text = text.trim();
            if text.is_empty() {
                continue;
            }
            let Ok(mut sc) = Shortcut::parse(text) else {
                continue;
            };
            // No explicit scope: keep the scope of the default binding.
            if (!text.contains(':') || text == ":")
                && let Some(d) = def.shortcut
            {
                sc.scope = d.scope;
            }
            map.bind(def.id, sc);
        }
        let mut defaults: Vec<(CmdId, Shortcut)> = Vec::new();
        for def in reg.commands() {
            if overridden.contains(&def.id) {
                continue;
            }
            defaults.extend(def.default_shortcuts().map(|s| (def.id, *s)));
        }
        defaults.sort_by_key(|(id, _)| *id);
        for (id, sc) in defaults {
            map.bind(id, sc);
        }
        map
    }

    fn bind(&mut self, cmd: CmdId, sc: Shortcut) {
        let slot = self.by_key.entry((sc.scope, sc.key)).or_default();
        if let Some(existing) = slot
            .iter()
            .find(|b| b.mods.matches_exact(sc.mods) && sc.mods.matches_exact(b.mods))
        {
            if existing.cmd != cmd {
                self.conflicts.push(ShortcutConflict {
                    scope: sc.scope,
                    shortcut: sc.display(),
                    kept: existing.cmd,
                    ignored: cmd,
                });
            }
            return;
        }
        slot.push(Binding { mods: sc.mods, cmd });
        self.by_cmd.entry(cmd).or_default().push(sc);
    }

    /// The command bound to `key`+`mods` in the highest active scope that binds it.
    /// `scopes` lists the active scopes in any order; priority is Sketch > Viewport > Global.
    pub fn resolve(
        &self,
        key: egui::Key,
        mods: egui::Modifiers,
        scopes: &[ShortcutScope],
    ) -> Option<CmdId> {
        for scope in ShortcutScope::BY_PRIORITY {
            if !scopes.contains(&scope) {
                continue;
            }
            if let Some(list) = self.by_key.get(&(scope, key))
                && let Some(b) = list.iter().find(|b| mods.matches_exact(b.mods))
            {
                return Some(b.cmd);
            }
        }
        None
    }

    /// Like [`Self::resolve`] but ignores bindings that are plain text keys (no Ctrl/Alt) when
    /// `text_focus` is set: single letters never fire while a text field has focus.
    pub fn resolve_for_input(
        &self,
        key: egui::Key,
        mods: egui::Modifiers,
        scopes: &[ShortcutScope],
        text_focus: bool,
    ) -> Option<CmdId> {
        if text_focus {
            // Only Ctrl/Alt chords may pass, and not the ones a text field edits with.
            let editing = matches!(
                key,
                egui::Key::Z
                    | egui::Key::Y
                    | egui::Key::A
                    | egui::Key::C
                    | egui::Key::V
                    | egui::Key::X
            );
            if !(mods.command || mods.ctrl || mods.alt) || editing {
                return None;
            }
        }
        self.resolve(key, mods, scopes)
    }

    /// The shortcuts of a command, the first one is the one shown in menus.
    pub fn shortcuts_of(&self, cmd: CmdId) -> &[Shortcut] {
        self.by_cmd.get(&cmd).map_or(&[], Vec::as_slice)
    }

    /// The text shown next to a command (`"Ctrl+S"`, `"L"`), if it has a shortcut.
    pub fn display_of(&self, cmd: CmdId) -> Option<String> {
        self.shortcuts_of(cmd).first().map(Shortcut::display)
    }

    /// Conflicts found while building (the later binding was ignored).
    pub fn conflicts(&self) -> &[ShortcutConflict] {
        &self.conflicts
    }

    /// Override ids that name no registered command.
    pub fn unknown_ids(&self) -> &[String] {
        &self.unknown
    }

    /// The registry generation this map was built for.
    pub fn generation(&self) -> u64 {
        self.generation
    }

    /// Number of bound shortcuts.
    pub fn len(&self) -> usize {
        self.by_cmd.values().map(Vec::len).sum()
    }

    /// True if nothing is bound.
    pub fn is_empty(&self) -> bool {
        self.by_cmd.is_empty()
    }
}

/// The default `shortcuts.toml` path: `%APPDATA%\Amalgam\shortcuts.toml` on Windows.
pub fn default_shortcuts_path() -> Option<std::path::PathBuf> {
    directories::BaseDirs::new().map(|d| d.config_dir().join("Amalgam").join("shortcuts.toml"))
}

/// The key presses of this frame, in order (`pressed`, not auto-repeat).
pub(crate) fn key_presses(ctx: &egui::Context) -> Vec<(egui::Key, egui::Modifiers)> {
    ctx.input(|i| {
        i.events
            .iter()
            .filter_map(|e| match e {
                egui::Event::Key {
                    key,
                    pressed: true,
                    repeat: false,
                    modifiers,
                    ..
                } => Some((*key, *modifiers)),
                _ => None,
            })
            .collect()
    })
}

/// Removes a consumed key press (and its text event for plain letters) from this frame's input
/// so widgets drawn later do not see it.
pub(crate) fn consume_press(ctx: &egui::Context, key: egui::Key, mods: egui::Modifiers) {
    ctx.input_mut(|i| {
        if let Some(pos) = i.events.iter().position(|e| {
            matches!(e, egui::Event::Key { key: k, pressed: true, modifiers: m, .. }
                if *k == key && *m == mods)
        }) {
            i.events.remove(pos);
        }
    });
}

#[cfg(test)]
mod tests {
    use super::*;
    use egui::{Key, Modifiers};

    #[test]
    fn parse_forms() {
        let s = Shortcut::parse("Ctrl+Shift+S").unwrap();
        assert_eq!(s.key, Key::S);
        assert_eq!(s.mods, Modifiers::COMMAND | Modifiers::SHIFT);
        assert_eq!(s.scope, ShortcutScope::Global);
        let v = Shortcut::parse("viewport:e").unwrap();
        assert_eq!((v.key, v.scope), (Key::E, ShortcutScope::Viewport));
        assert_eq!(v.mods, Modifiers::NONE);
        let k = Shortcut::parse("sketch:Alt+Escape").unwrap();
        assert_eq!((k.key, k.scope), (Key::Escape, ShortcutScope::Sketch));
        assert!(k.mods.alt);
        assert_eq!(Shortcut::parse("F5").unwrap().key, Key::F5);
        assert_eq!(Shortcut::parse("Ctrl+Z").unwrap().display(), "Ctrl+Z");
    }

    #[test]
    fn parse_errors() {
        assert_eq!(Shortcut::parse(""), Err(ShortcutParseError::Empty));
        assert!(matches!(
            Shortcut::parse("moon:L"),
            Err(ShortcutParseError::Scope(_))
        ));
        assert!(matches!(
            Shortcut::parse("Hyper+L"),
            Err(ShortcutParseError::Modifier(_))
        ));
        assert!(matches!(
            Shortcut::parse("Ctrl+Banana"),
            Err(ShortcutParseError::Key(_))
        ));
    }

    #[test]
    fn text_roundtrip_and_serde() {
        for t in ["Ctrl+Shift+S", "sketch:L", "viewport:Alt+E", "Escape"] {
            let s = Shortcut::parse(t).unwrap();
            assert_eq!(Shortcut::parse(&s.to_text()).unwrap(), s, "{t}");
        }
        let s = Shortcut::parse("sketch:Ctrl+L").unwrap();
        let json = serde_json::to_string(&s).unwrap();
        assert_eq!(json, "\"sketch:Ctrl+L\"");
        assert_eq!(serde_json::from_str::<Shortcut>(&json).unwrap(), s);
    }

    #[test]
    fn overrides_file_parses_and_rejects_typos() {
        let o = ShortcutOverrides::parse("[shortcuts]\n\"a.b\" = \"Ctrl+K\"\n\"c.d\" = \"\"\n")
            .unwrap();
        assert_eq!(o.len(), 2);
        assert!(ShortcutOverrides::parse("[shortcuts]\n\"a.b\" = \"Ctrl+Nope\"\n").is_err());
        assert!(ShortcutOverrides::parse("not toml [").is_err());
        assert!(ShortcutOverrides::parse("").unwrap().is_empty());
        let back = ShortcutOverrides::parse(&o.to_toml()).unwrap();
        assert_eq!(back, o);
    }

    #[test]
    fn scopes_by_priority() {
        assert_eq!(
            active_scopes(true, true),
            [
                ShortcutScope::Sketch,
                ShortcutScope::Viewport,
                ShortcutScope::Global
            ]
        );
        assert_eq!(active_scopes(false, false), [ShortcutScope::Global]);
    }
}
