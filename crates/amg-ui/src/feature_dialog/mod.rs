//! Generic feature dialogs (spec §9 "UI-05 generic renderer", note `ui-framework.md` §6).
//!
//! Every [`DialogRegistration`] of `amg_features::dialog` becomes, without UI code of its own:
//! - a command `feature.<kind>` ([`command_id`]) placed in the toolbar group
//!   `<workspace>.<tab>.<group>` of its [`CommandMeta`], with its icon, shortcut and search
//!   words;
//! - a [`FeatureUiInfo`] (display name = the schema title, [`EditAction::GenericDialog`]), so
//!   the timeline, the browser and "Edit Feature" open the same dialog in edit mode;
//! - a [`GenericFeatureTool`] when the command runs: the schema's fields rendered with the
//!   UI-15 widgets, `visible_if` / `enabled_if`, auto-focus of the first empty selection field,
//!   per-field errors with OK disabled, a debounced latest-wins live preview through the
//!   document's [`PreviewDriver`](crate::preview::PreviewDriver) (a job; never on the UI
//!   thread) and its health as the dialog banner.
//!
//! A [`DispatchRegistration`] becomes a command that opens the dialog of the kind its
//! `dispatch` picks for the selection.
//!
//! Registration runs after every module ([`register_generic`], from
//! [`UiRegistry::build`]): a module that registers its own `feature.<kind>` command or
//! timeline info for a kind keeps it.
//!
//! **Edit mode** (`ToolInit::edit`): the fields start from `read`. The preview is an
//! `EditFeature` on the preview's scratch copy (bound expression fields that changed are
//! previewed inline), so Cancel leaves the document untouched. OK commits one batch: the
//! `EditFeature` (bound fields keep their model parameter) and a `SetParamExpr` for every
//! bound field whose text changed, so a timeline dimension `d3` stays `d3`.

mod profiles;
mod tool;

pub use tool::GenericFeatureTool;

use std::collections::HashMap;
use std::sync::{Arc, Mutex, OnceLock};

use amg_features::dialog::{self, CommandMeta, DialogRegistration, DispatchRegistration};

use crate::command::{CmdAction, CmdId, CommandDef, Enablement, Enabler, Placement};
use crate::module::{GroupId, IconId, Registrar};
use crate::registry::UiRegistry;
use crate::timeline::{EditAction, FeatureUiInfo};
use crate::tool::{Tool, ToolInit};

/// The registry module name of the generated entries.
pub const MODULE: &str = "feature";

/// Prefix of every generated command id.
pub const COMMAND_PREFIX: &str = "feature.";

/// Icon of registrations without an SVG.
pub const FALLBACK_ICON: IconId = crate::icons::ids::CUBE_PLUS;

/// Interns `s` for the process lifetime: registry ids are `&'static str`, and the registry is
/// rebuilt per app, so each distinct string is leaked once.
fn intern(s: String) -> &'static str {
    static TABLE: OnceLock<Mutex<HashMap<String, &'static str>>> = OnceLock::new();
    let mut t = TABLE
        .get_or_init(Default::default)
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner);
    if let Some(v) = t.get(&s) {
        return v;
    }
    let v: &'static str = Box::leak(s.clone().into_boxed_str());
    t.insert(s, v);
    v
}

/// The command of a feature kind: `feature.<kind>`.
pub fn command_id(kind: &str) -> CmdId {
    CmdId(intern(format!("{COMMAND_PREFIX}{kind}")))
}

/// The feature kind of a generated command id (`feature.<kind>` → `<kind>`).
pub fn kind_of(cmd: CmdId) -> Option<&'static str> {
    cmd.0.strip_prefix(COMMAND_PREFIX)
}

/// The toolbar group of a command placement: `<workspace>.<tab>.<group>` (the Design
/// workspace's convention, `design.<tab>.<group>`).
pub fn group_of(meta: &CommandMeta) -> GroupId {
    GroupId(intern(format!(
        "{}.{}.{}",
        meta.workspace, meta.tab, meta.group
    )))
}

/// The icon id of a registration: its own SVG under `feature.<kind>`, else [`FALLBACK_ICON`].
pub fn icon_of(kind: &str, meta: &CommandMeta) -> IconId {
    if meta.icon_svg.is_empty() {
        FALLBACK_ICON
    } else {
        IconId(intern(format!("{COMMAND_PREFIX}{kind}")))
    }
}

/// The command definition of a registration (toolbar, search, shortcut; starts
/// [`GenericFeatureTool`]).
pub fn command_def(reg: &'static DialogRegistration) -> CommandDef {
    let schema = (reg.schema)();
    let def = CommandDef::new(
        command_id(reg.kind).0,
        schema.title.clone(),
        CmdAction::StartTool(start),
    );
    finish_def(def, reg.kind, &reg.command).with_tooltip(schema.title.clone())
}

/// The timeline info of a registration.
pub fn feature_ui_info(reg: &'static DialogRegistration) -> FeatureUiInfo {
    FeatureUiInfo {
        kind: reg.kind,
        display_name: intern((reg.schema)().title.clone()),
        icon: icon_of(reg.kind, &reg.command),
        edit: EditAction::GenericDialog,
    }
}

fn finish_def(mut def: CommandDef, kind: &str, meta: &CommandMeta) -> CommandDef {
    def = def
        .with_icon(icon_of(kind, meta))
        .placed(Placement::Toolbar {
            group: group_of(meta),
            pinned: meta.pinned,
            order: meta.order,
        })
        .with_keywords(meta.keywords);
    if let Some(s) = meta.shortcut {
        def = def.with_shortcut(s);
    }
    def
}

/// The command of a dispatch registration: enabled while `dispatch` finds a kind for the
/// selection; it opens that kind's dialog.
pub fn dispatch_def(d: &'static DispatchRegistration) -> CommandDef {
    let def = CommandDef::new(d.command_id, d.command_id, CmdAction::StartTool(start))
        .with_enabler(Enabler::Dyn(Arc::new(move |env| {
            let refs: Vec<amg_doc::EntityRef> = env
                .selection
                .items()
                .iter()
                .map(|i| i.entity.clone())
                .collect();
            match (d.dispatch)(&refs, env.design) {
                Some(_) => Enablement::Enabled,
                None => Enablement::disabled("Nothing in the selection fits this command"),
            }
        })));
    finish_def(def, d.command_id, &d.command)
}

/// Adds the generated commands, icons and timeline infos to `reg` (runs after every module;
/// entries a module registered itself are kept).
pub(crate) fn register_generic(reg: &mut UiRegistry) {
    let mut r = Registrar {
        reg,
        module: MODULE,
    };
    for d in dialog::registrations() {
        let cmd = command_id(d.kind);
        let icon = icon_of(d.kind, &d.command);
        if !d.command.icon_svg.is_empty() && !r.reg.has_icon(icon) {
            r.icon(icon, d.command.icon_svg);
        }
        if r.reg.command(cmd).is_none() {
            r.command(command_def(d));
        }
        if r.reg.feature_ui(d.kind).is_none() {
            r.feature_ui(feature_ui_info(d));
        }
    }
    for d in dialog::dispatches() {
        let icon = icon_of(d.command_id, &d.command);
        if !d.command.icon_svg.is_empty() && !r.reg.has_icon(icon) {
            r.icon(icon, d.command.icon_svg);
        }
        if r.reg.command(CmdId(d.command_id)).is_none() {
            r.command(dispatch_def(d));
        }
    }
}

/// The `StartTool` factory of every generated command: the dialog of the kind named by
/// `init.command` (`feature.<kind>`, or a dispatch command), or of the edited feature's kind.
pub fn start(init: &ToolInit<'_>) -> Box<dyn Tool> {
    let kind: Option<String> = match init.edit {
        Some(id) => init
            .env
            .design
            .timeline()
            .node(id)
            .map(|(_, n)| n.feature.kind().to_owned()),
        None => match kind_of(init.command) {
            Some(k) => Some(k.to_owned()),
            None => dialog::dispatches()
                .find(|d| d.command_id == init.command.0)
                .and_then(|d| {
                    let refs: Vec<amg_doc::EntityRef> =
                        init.preselection.iter().map(|i| i.entity.clone()).collect();
                    (d.dispatch)(&refs, init.env.design).map(str::to_owned)
                }),
        },
    };
    let Some(reg) = kind.as_deref().and_then(dialog::registration) else {
        return Box::new(tool::Unavailable::new(
            init.command,
            format!(
                "No dialog is registered for `{}`.",
                kind.as_deref().unwrap_or(init.command.0)
            ),
        ));
    };
    match GenericFeatureTool::new(reg, init) {
        Ok(t) => Box::new(t),
        Err(msg) => Box::new(tool::Unavailable::new(init.command, msg)),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ids_are_derived_from_the_registration() {
        assert_eq!(command_id("a.b"), CmdId("feature.a.b"));
        assert!(std::ptr::eq(command_id("a.b").0, command_id("a.b").0));
        assert_eq!(kind_of(CmdId("feature.a.b")), Some("a.b"));
        assert_eq!(kind_of(CmdId("mesh.remesh")), None);
        let meta = CommandMeta {
            workspace: "design",
            tab: "t",
            group: "create",
            order: 0,
            pinned: false,
            icon_svg: &[],
            shortcut: None,
            keywords: &[],
        };
        assert_eq!(group_of(&meta), GroupId("design.t.create"));
        assert_eq!(icon_of("a.b", &meta), FALLBACK_ICON);
        let with_svg = CommandMeta {
            icon_svg: b"<svg/>",
            ..meta
        };
        assert_eq!(icon_of("a.b", &with_svg), IconId("feature.a.b"));
    }
}
